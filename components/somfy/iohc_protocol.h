#pragma once

// io-homecontrol protocol core — pure, dependency-free helpers.
//
// This file intentionally pulls in nothing from ESPHome or mbedTLS so the
// framing/CRC/checksum/IV/MAC logic can be unit-tested on a host machine
// against captured golden vectors (see tests/cpp/test_iohc_protocol.cpp).
// AES is injected by the caller: firmware supplies the mbedTLS wrapper, the
// host test supplies a known-correct AES-128 oracle.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace somfy {
namespace iohc_proto {

// AES-128 ECB single-block encrypt: out = AES_enc(key, in).
using Aes128EcbFn = void (*)(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

// CRC-16 used by io-homecontrol (poly 0x8408, init 0x0000, reflected, LSB-first
// on the wire). Computed over [data, data+len). Append as two bytes,
// low byte first; a frame including its own CRC then checksums to 0x0000.
uint16_t crc16(const uint8_t *data, size_t len);

// io-homecontrol rolling checksum over [data, data+len). Returns the two
// accumulator bytes (chk1 = high, chk2 = low) that feed the MAC IV.
void rolling_checksum(const uint8_t *data, size_t len, uint8_t &chk1, uint8_t &chk2);

// Build the 16-byte AES IV for a 1W MAC.
//   payload = cmd || data (the command payload, excluding node addresses)
//   iv[0..7]   = first 8 payload bytes, 0x55-padded if payload shorter than 8
//   iv[8..9]   = rolling checksum over the full payload
//   iv[10..11] = sequence number (big-endian)
//   iv[12..15] = 0x55 padding
void build_iv_1w(const uint8_t *payload, size_t payload_len, uint16_t sequence, uint8_t iv[16]);

// Build the 16-byte AES IV for a 2W challenge response.
//   payload = cmd || data (the command payload, excluding node addresses)
//   iv[0..7]   = first 8 payload bytes, 0x55-padded if payload shorter than 8
//   iv[8..9]   = rolling checksum over the full payload
//   iv[10..15] = 6-byte challenge supplied by the actuator
void build_iv_2w(const uint8_t *payload, size_t payload_len, const uint8_t challenge[6], uint8_t iv[16]);

// MAC = AES(key, iv) truncated to the leading 6 bytes.
void compute_mac(Aes128EcbFn aes, const uint8_t key[16], const uint8_t iv[16], uint8_t mac_out[6]);

// 1W controller-key obfuscation for the 0x30 key-transfer frame.
//   keystream = AES(transfer_key, IV) where IV is the 3-byte node address
//   (big-endian) repeated to fill all 16 bytes.
//   enc_out[i] = plain_key[i] XOR keystream[i].
void obfuscate_key_1w(Aes128EcbFn aes, const uint8_t transfer_key[16], uint32_t node_addr,
                      const uint8_t plain_key[16], uint8_t enc_out[16]);

// Build a complete ordinary logical 1W frame (ctrl0 .. CRC). The MAC
// authenticates cmd || data[0..auth_len), and the sequence is placed between
// data and MAC. Key transfer (cmd 0x30) has a special no-MAC wire format; use
// build_key_transfer_frame_1w() for that command.
bool build_frame_1w(Aes128EcbFn aes, const uint8_t key[16], uint32_t src_node,
                    uint32_t dest_node, uint8_t cmd, const uint8_t *data,
                    size_t data_len, size_t auth_len, uint16_t sequence,
                    std::vector<uint8_t> &out);

// Build the 31-byte cmd 0x30 install-key management frame:
//   FC ctrl1 dst[3] src[3] 30 enc_key[16] manufacturer data seq[2] crc[2]
bool build_key_transfer_frame_1w(uint32_t src_node, uint32_t dest_node,
                                 const uint8_t encrypted_key[16],
                                 uint8_t manufacturer, uint8_t data,
                                 uint16_t sequence, uint8_t ctrl1,
                                 std::vector<uint8_t> &out);

// Extract the big-endian rolling sequence from the decoded data portion of an
// ordinary 1W frame. The final eight bytes are sequence[2] || MAC[6], regardless
// of the command payload length.
bool extract_sequence_1w(const uint8_t *data, size_t data_len, uint16_t &sequence);

// Decide whether a direction/STOP-MY terminal event completes a previously
// staged D200 prefix while the caller's short gesture window is still open.
// When both frames expose rolling sequences, a real terminal consumes exactly
// the next sequence. Sequence-less captures fall back to the already-bounded
// same-remote time correlation rather than inventing a sequence relation.
bool gesture_terminal_matches_prefix(uint32_t prefix_remote,
                                     uint16_t prefix_sequence,
                                     bool prefix_has_sequence,
                                     uint32_t terminal_remote,
                                     uint16_t terminal_sequence,
                                     bool terminal_has_sequence);

// Collapses repeated RF copies of one logical frame while preserving rapid new
// presses of the same button. Sequence-aware frames compare their rolling
// sequence; legacy/fallback frames compare their main parameter.
class RxBurstDeduplicator {
 public:
  bool is_duplicate(uint32_t now_ms, uint32_t src, uint16_t main_param,
                    uint16_t sequence, bool has_sequence, uint32_t window_ms);

 private:
  uint32_t src_{0};
  uint16_t main_param_{0};
  uint16_t sequence_{0};
  uint32_t seen_ms_{0};
  bool has_sequence_{false};
  bool valid_{false};
};

// --- Physical layer (UART-8N1) codec -------------------------------------
//
// io-homecontrol modulates 2-FSK at 38400 baud, but the bit stream is *not*
// raw NRZ: every logical byte is wrapped in a UART 8N1 frame — a 0 start bit,
// 8 data bits transmitted LSB-first, then a 1 stop bit — and bytes are sent in
// order. The on-air message is:
//
//   preamble (256 bits of 0101…, i.e. UART-framed 0x55 bytes)
//     + sync   (UART-framed 0xFF 0x33  ->  bits 0111111111 0110011001)
//     + payload (each logical frame byte UART-framed)
//
// The CC1101 hardware can generate the alternating preamble and match a 16-bit
// sync word, but it cannot UART-frame the payload. We therefore co-opt the
// hardware: program a preamble-tail-aligned sync word (0x57FD), and let
// software handle everything after it. The remaining 8 bits of UART-framed
// 0x33 (0x99) sit at the head of the FIFO payload, immediately before the
// first logical byte's UART frame. This alignment is validated on CC1101
// hardware for both RX and TX.
//
// build_iv_1w / crc16 / etc. operate on the *logical* bytes (start/stop bits
// stripped, bit order restored) — exactly what the documented captures show.

// Four preamble bits, the complete UART-framed 0xFF, and the first two bits of
// UART-framed 0x33. Program sync_mode 16/16 at runtime.
static constexpr uint8_t PHY_HW_SYNC1 = 0x57;
static constexpr uint8_t PHY_HW_SYNC0 = 0xFD;

// Remaining UART-framed 0x33 bits after the 0x57FD hardware sync match.
static constexpr uint8_t PHY_SYNC_RESIDUE = 0x99;
static constexpr uint8_t PHY_SYNC_RESIDUE_BITS = 8;

// Encode a logical frame (ctrl0 .. crc) into the CC1101 FIFO payload that
// follows the 16-bit hardware sync word. The payload is the 8-bit encoded-sync
// residue (0x99) followed by each logical byte UART-framed (start 0, 8 data bits
// LSB-first, stop 1), bit-packed MSB-first. Any partial trailing byte is padded
// with idle (1) bits. The CC1101 must be in fixed-length packet mode with the
// length set to out.size() and hardware CRC disabled.
void uart_encode(const uint8_t *logical, size_t len, std::vector<uint8_t> &out);

// Decode a CC1101 FIFO payload (captured after a 0x57FD sync match) back into
// logical frame bytes. Skips the 8-bit encoded-sync residue, then UART-decodes
// each following 10-bit group. Stops at the first framing error (bad start/stop
// bit) or when fewer than 10 bits remain. Returns the number of bytes decoded.
size_t uart_decode(const uint8_t *payload, size_t len, std::vector<uint8_t> &out);

}  // namespace iohc_proto
}  // namespace somfy
}  // namespace esphome
