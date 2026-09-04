#include "iohc_protocol.h"

#include <cstring>

namespace esphome {
namespace somfy {
namespace iohc_proto {

namespace {
constexpr uint16_t CRC_POLY = 0x8408;
constexpr uint16_t CRC_INIT = 0x0000;
constexpr uint8_t IV_PAD = 0x55;

// Append bits MSB-first into a growing byte buffer.
class BitWriter {
 public:
  explicit BitWriter(std::vector<uint8_t> &out) : out_(out) {}
  void put(uint8_t bit) {
    if (count_ % 8 == 0)
      out_.push_back(0);
    if (bit & 1)
      out_.back() |= static_cast<uint8_t>(1u << (7 - (count_ % 8)));
    count_++;
  }
  // Fill the remainder of the current byte with the given idle bit.
  void pad_to_byte(uint8_t fill_bit) {
    while (count_ % 8 != 0)
      put(fill_bit);
  }

 private:
  std::vector<uint8_t> &out_;
  size_t count_{0};
};

// Read bits MSB-first from a byte buffer.
class BitReader {
 public:
  BitReader(const uint8_t *data, size_t len) : data_(data), total_(len * 8) {}
  bool get(uint8_t &bit) {
    if (pos_ >= total_)
      return false;
    bit = (data_[pos_ >> 3] >> (7 - (pos_ & 7))) & 1;
    pos_++;
    return true;
  }
  void skip(size_t n) { pos_ += n; }
  size_t remaining() const { return total_ > pos_ ? total_ - pos_ : 0; }

 private:
  const uint8_t *data_;
  size_t total_;
  size_t pos_{0};
};
}  // namespace

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = CRC_INIT;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ CRC_POLY;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void rolling_checksum(const uint8_t *data, size_t len, uint8_t &chk1, uint8_t &chk2) {
  chk1 = 0;
  chk2 = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t tmp = data[i] ^ chk2;
    chk2 = static_cast<uint8_t>(((chk1 & 0x7F) << 1) & 0xFF);
    if (tmp >= 0x80)
      chk2 |= 1;
    if ((chk1 & 0x80) == 0) {
      chk1 = chk2;
      chk2 = static_cast<uint8_t>((tmp << 1) & 0xFF);
    } else {
      chk1 = static_cast<uint8_t>(chk2 ^ 0x55);
      chk2 = static_cast<uint8_t>(((tmp << 1) ^ 0x5B) & 0xFF);
    }
  }
}

static void build_iv_head(const uint8_t *payload, size_t payload_len, uint8_t iv[16]) {
  memset(iv, IV_PAD, 16);
  const size_t head = (payload_len > 8) ? 8 : payload_len;
  if (head > 0)
    memcpy(iv, payload, head);
  rolling_checksum(payload, payload_len, iv[8], iv[9]);
}

void build_iv_1w(const uint8_t *payload, size_t payload_len, uint16_t sequence, uint8_t iv[16]) {
  build_iv_head(payload, payload_len, iv);
  iv[10] = static_cast<uint8_t>(sequence >> 8);
  iv[11] = static_cast<uint8_t>(sequence & 0xFF);
  iv[12] = IV_PAD;
  iv[13] = IV_PAD;
  iv[14] = IV_PAD;
  iv[15] = IV_PAD;
}

void build_iv_2w(const uint8_t *payload, size_t payload_len, const uint8_t challenge[6], uint8_t iv[16]) {
  build_iv_head(payload, payload_len, iv);
  memcpy(iv + 10, challenge, 6);
}

void compute_mac(Aes128EcbFn aes, const uint8_t key[16], const uint8_t iv[16], uint8_t mac_out[6]) {
  uint8_t encrypted[16];
  aes(key, iv, encrypted);
  memcpy(mac_out, encrypted, 6);
}

void obfuscate_key_1w(Aes128EcbFn aes, const uint8_t transfer_key[16], uint32_t node_addr,
                      const uint8_t plain_key[16], uint8_t enc_out[16]) {
  const uint8_t addr[3] = {
      static_cast<uint8_t>((node_addr >> 16) & 0xFF),
      static_cast<uint8_t>((node_addr >> 8) & 0xFF),
      static_cast<uint8_t>(node_addr & 0xFF),
  };
  uint8_t iv[16];
  for (size_t i = 0; i < 16; i++)
    iv[i] = addr[i % 3];

  uint8_t keystream[16];
  aes(transfer_key, iv, keystream);
  for (size_t i = 0; i < 16; i++)
    enc_out[i] = static_cast<uint8_t>(plain_key[i] ^ keystream[i]);
}

bool build_frame_1w(Aes128EcbFn aes, const uint8_t key[16], uint32_t src_node,
                    uint32_t dest_node, uint8_t cmd, const uint8_t *data,
                    size_t data_len, size_t auth_len, uint16_t sequence,
                    std::vector<uint8_t> &out) {
  constexpr size_t MAC_LEN = 6;
  constexpr uint8_t CTRL0_1W = 0xE0;

  out.clear();
  if (aes == nullptr || key == nullptr || auth_len > data_len ||
      (data_len != 0 && data == nullptr))
    return false;

  out.reserve(2 + 6 + 1 + data_len + 2 + MAC_LEN + 2);
  out.push_back(0x00);  // ctrl0 placeholder
  out.push_back(0x00);  // ctrl1

  out.push_back(static_cast<uint8_t>((dest_node >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((dest_node >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(dest_node & 0xFF));
  out.push_back(static_cast<uint8_t>((src_node >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((src_node >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(src_node & 0xFF));
  out.push_back(cmd);
  if (data_len > 0)
    out.insert(out.end(), data, data + data_len);
  out.push_back(static_cast<uint8_t>(sequence >> 8));
  out.push_back(static_cast<uint8_t>(sequence & 0xFF));

  std::vector<uint8_t> authenticated;
  authenticated.reserve(1 + auth_len);
  authenticated.push_back(cmd);
  if (auth_len > 0)
    authenticated.insert(authenticated.end(), data, data + auth_len);

  uint8_t iv[16];
  build_iv_1w(authenticated.data(), authenticated.size(), sequence, iv);
  uint8_t mac[MAC_LEN];
  compute_mac(aes, key, iv, mac);
  out.insert(out.end(), mac, mac + MAC_LEN);

  const size_t declared_size = out.size() - 1;
  if (declared_size > 0x1F) {
    out.clear();
    return false;
  }
  out[0] = static_cast<uint8_t>(CTRL0_1W | declared_size);

  const uint16_t crc = crc16(out.data(), out.size());
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>(crc >> 8));
  return true;
}

bool build_key_transfer_frame_1w(uint32_t src_node, uint32_t dest_node,
                                 const uint8_t encrypted_key[16],
                                 uint8_t manufacturer, uint8_t data,
                                 uint16_t sequence, uint8_t ctrl1,
                                 std::vector<uint8_t> &out) {
  constexpr uint8_t CMD_WRITE_PRIVATE = 0x30;
  constexpr uint8_t CTRL0_1W = 0xE0;
  constexpr size_t DECLARED_SIZE = 28;

  out.clear();
  if (encrypted_key == nullptr)
    return false;

  out.reserve(31);
  out.push_back(static_cast<uint8_t>(CTRL0_1W | DECLARED_SIZE));
  out.push_back(ctrl1);
  out.push_back(static_cast<uint8_t>((dest_node >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((dest_node >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(dest_node & 0xFF));
  out.push_back(static_cast<uint8_t>((src_node >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((src_node >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(src_node & 0xFF));
  out.push_back(CMD_WRITE_PRIVATE);
  out.insert(out.end(), encrypted_key, encrypted_key + 16);
  out.push_back(manufacturer);
  out.push_back(data);
  out.push_back(static_cast<uint8_t>(sequence >> 8));
  out.push_back(static_cast<uint8_t>(sequence & 0xFF));

  const uint16_t crc = crc16(out.data(), out.size());
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>(crc >> 8));
  return true;
}

bool extract_sequence_1w(const uint8_t *data, size_t data_len, uint16_t &sequence) {
  constexpr size_t SEQUENCE_AND_MAC_LEN = 8;
  if (data == nullptr || data_len < SEQUENCE_AND_MAC_LEN)
    return false;
  const size_t offset = data_len - SEQUENCE_AND_MAC_LEN;
  sequence = (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1];
  return true;
}

bool gesture_terminal_matches_prefix(uint32_t prefix_remote,
                                     uint16_t prefix_sequence,
                                     bool prefix_has_sequence,
                                     uint32_t terminal_remote,
                                     uint16_t terminal_sequence,
                                     bool terminal_has_sequence) {
  if ((prefix_remote & 0x00FFFFFFU) !=
      (terminal_remote & 0x00FFFFFFU)) {
    return false;
  }
  if (!prefix_has_sequence || !terminal_has_sequence)
    return true;
  return terminal_sequence == static_cast<uint16_t>(prefix_sequence + 1U);
}

bool RxBurstDeduplicator::is_duplicate(uint32_t now_ms, uint32_t src, uint16_t main_param,
                                       uint16_t sequence, bool has_sequence, uint32_t window_ms) {
  const bool same_frame = has_sequence && this->has_sequence_
                              ? sequence == this->sequence_
                              : !has_sequence && !this->has_sequence_ && main_param == this->main_param_;
  if (this->valid_ && src == this->src_ && same_frame && (now_ms - this->seen_ms_) < window_ms) {
    this->seen_ms_ = now_ms;  // extend the window across the whole RF burst
    return true;
  }

  this->valid_ = true;
  this->src_ = src;
  this->main_param_ = main_param;
  this->sequence_ = sequence;
  this->has_sequence_ = has_sequence;
  this->seen_ms_ = now_ms;
  return false;
}

void uart_encode(const uint8_t *logical, size_t len, std::vector<uint8_t> &out) {
  out.clear();
  BitWriter bw(out);
  // Eight-bit encoded-sync residue after the preamble-aligned 0x57FD hardware
  // sync. This is the remainder of the UART-framed logical 0x33 byte.
  for (uint8_t k = 0; k < 8; k++)
    bw.put((PHY_SYNC_RESIDUE >> (7 - k)) & 1);
  for (size_t i = 0; i < len; i++) {
    bw.put(0);  // start bit
    for (uint8_t k = 0; k < 8; k++)
      bw.put((logical[i] >> k) & 1);  // data bits, LSB first
    bw.put(1);                        // stop bit
  }
  bw.pad_to_byte(1);  // idle/mark padding for the trailing partial byte
}

size_t uart_decode(const uint8_t *payload, size_t len, std::vector<uint8_t> &out) {
  out.clear();
  BitReader br(payload, len);
  br.skip(PHY_SYNC_RESIDUE_BITS);
  while (br.remaining() >= 10) {
    uint8_t start = 0;
    br.get(start);
    uint8_t b = 0;
    for (uint8_t k = 0; k < 8; k++) {
      uint8_t bit = 0;
      br.get(bit);
      b |= static_cast<uint8_t>((bit & 1) << k);
    }
    uint8_t stop = 0;
    br.get(stop);
    if (start != 0 || stop != 1)
      break;  // framing error -> end of frame
    out.push_back(b);
  }
  return out.size();
}

}  // namespace iohc_proto
}  // namespace somfy
}  // namespace esphome
