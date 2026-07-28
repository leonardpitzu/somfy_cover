#include "somfy_iohc.h"

#ifdef USE_SOMFY_IOHC

#include "iohc_protocol.h"
#include "esphome/core/log.h"
#include <cstring>

#ifdef USE_SOMFY_IOHC_RX
#include "esphome/components/text_sensor/text_sensor.h"
#include <cinttypes>
#include <cmath>
#include <cstdio>
#endif

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.iohc";

#ifdef USE_SOMFY_IOHC_RX
namespace {
// Window over which an identical (src, main_param) command is treated as part of
// the remote's repeat burst rather than a fresh press.
constexpr uint32_t RX_DEDUP_WINDOW_MS = 1500;
// Throttle for the RX-driven position animation: at most one state publish per
// interval, and only when the position actually moved.
constexpr uint32_t RX_SYNC_PUBLISH_INTERVAL_MS = 250;
// Cap how many payload bytes we render to hex (foreign EXECUTE frames are short).
constexpr size_t RX_HEX_MAX_BYTES = 16;

const char *main_param_name(uint16_t mp) {
  switch (mp) {
    case iohc_cmd::MP_OPEN:  return "OPEN";
    case iohc_cmd::MP_CLOSE: return "CLOSE";
    case iohc_cmd::MP_STOP:  return "STOP";
    case iohc_cmd::MP_MY:    return "MY";
    default:                 return "POS";
  }
}

// Render up to RX_HEX_MAX_BYTES of a payload as "AA BB CC" into out (NUL-terminated).
void format_payload_hex(const uint8_t *data, size_t len, char *out, size_t out_size) {
  if (out_size == 0) return;
  out[0] = '\0';
  if (data == nullptr) return;
  const size_t cap = (len > RX_HEX_MAX_BYTES) ? RX_HEX_MAX_BYTES : len;
  size_t pos = 0;
  for (size_t i = 0; i < cap && pos + 3 < out_size; i++) {
    pos += snprintf(out + pos, out_size - pos, "%02X ", data[i]);
  }
  if (pos > 0)
    out[pos - 1] = '\0';  // drop trailing space
}
}  // namespace
#endif

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void SomfyIohcCover::set_encryption_key(const char *hex_key) {
  if (hex_key == nullptr || strlen(hex_key) < 32) {
    ESP_LOGE(TAG, "Encryption key must be 32 hex characters (16 bytes)");
    return;
  }
  for (int i = 0; i < 16; i++) {
    char byte_str[3] = {hex_key[i * 2], hex_key[i * 2 + 1], 0};
    this->encryption_key_[i] = static_cast<uint8_t>(strtol(byte_str, nullptr, 16));
  }
  this->has_custom_key_ = true;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void SomfyIohcCover::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Somfy iohc cover...");

  this->storage_ = std::make_unique<NVSRollingCodeStorage>(this->storage_namespace_, this->storage_key_);

  if (!this->has_custom_key_) {
    memcpy(this->encryption_key_, iohc_keys::TRANSFER_KEY, 16);
  }

  // Register RX callback on hub
  this->hub_->register_rx_callback([this](const IohcDecodedPacket &pkt) {
    this->on_iohc_packet_(pkt);
  });

  // Wire up time-based cover triggers
  this->open_action_ = std::make_unique<SomfyIohcAction<>>([this] { this->open(); });
  this->open_automation_ = std::make_unique<Automation<>>(this->get_open_trigger());
  this->open_automation_->add_action(this->open_action_.get());

  this->close_action_ = std::make_unique<SomfyIohcAction<>>([this] { this->close(); });
  this->close_automation_ = std::make_unique<Automation<>>(this->get_close_trigger());
  this->close_automation_->add_action(this->close_action_.get());

  this->stop_action_ = std::make_unique<SomfyIohcAction<>>([this] { this->stop(); });
  this->stop_automation_ = std::make_unique<Automation<>>(this->get_stop_trigger());
  this->stop_automation_->add_action(this->stop_action_.get());

  this->prog_button_->add_on_press_callback([this]() { this->program(); });

  this->has_built_in_endstop_ = true;
  this->assumed_state_ = true;

  SomfyTimeBasedCover::setup();
}

void SomfyIohcCover::dump_config() {
  ESP_LOGCONFIG(TAG, "Somfy iohc cover:");
  ESP_LOGCONFIG(TAG, "  Node ID: 0x%06" PRIX32, this->node_id_);
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->mode_ == IohcMode::MODE_2W ? "2W (bidirectional)" : "1W (broadcast)");
  if (this->mode_ == IohcMode::MODE_2W) {
    ESP_LOGCONFIG(TAG, "  Target node: 0x%06" PRIX32, this->target_node_);
  }
  ESP_LOGCONFIG(TAG, "  Storage: %s/%s", this->storage_namespace_, this->storage_key_);
  ESP_LOGCONFIG(TAG, "  Custom key: %s", this->has_custom_key_ ? "yes" : "no (transfer key)");
#ifdef USE_SOMFY_IOHC_RX
  ESP_LOGCONFIG(TAG, "  RX state-sync: enabled (%u allowed remote(s)%s)",
                static_cast<unsigned>(this->receive_remote_codes_.size()),
                this->receive_remote_codes_.empty() ? ", accept-all" : "");
#endif
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

void SomfyIohcCover::open() {
  ESP_LOGD(TAG, "OPEN node=0x%06" PRIX32 " mode=%s", this->node_id_,
           this->mode_ == IohcMode::MODE_2W ? "2W" : "1W");
  if (this->mode_ == IohcMode::MODE_2W)
    this->send_2w_command(iohc_cmd::MP_OPEN);
  else
    this->send_1w_command(iohc_cmd::MP_OPEN);
}

void SomfyIohcCover::close() {
  ESP_LOGD(TAG, "CLOSE node=0x%06" PRIX32 " mode=%s", this->node_id_,
           this->mode_ == IohcMode::MODE_2W ? "2W" : "1W");
  if (this->mode_ == IohcMode::MODE_2W)
    this->send_2w_command(iohc_cmd::MP_CLOSE);
  else
    this->send_1w_command(iohc_cmd::MP_CLOSE);
}

void SomfyIohcCover::stop() {
  ESP_LOGD(TAG, "STOP node=0x%06" PRIX32 " mode=%s", this->node_id_,
           this->mode_ == IohcMode::MODE_2W ? "2W" : "1W");
  if (this->mode_ == IohcMode::MODE_2W)
    this->send_2w_command(iohc_cmd::MP_STOP);
  else
    this->send_1w_command(iohc_cmd::MP_STOP);
}

void SomfyIohcCover::program() {
  ESP_LOGI(TAG, "PROG (pair): node=0x%06" PRIX32 " -> dest=BROADCAST(0x%06" PRIX32 ") repeat=%d", this->node_id_,
           static_cast<uint32_t>(iohc::BROADCAST_ADDR), this->repeat_count_);

  // Step 1: CMD_REMOVE_CONTROLLER (0x39) carries a single data byte (0x00).
  uint8_t remove_data[1] = {0x00};
  auto frame_remove =
      this->build_1w_frame(iohc_cmd::CMD_REMOVE_CONTROLLER, remove_data, sizeof(remove_data), iohc::BROADCAST_ADDR);
  ESP_LOGD(TAG, "PROG: tx CMD_REMOVE_CONTROLLER (0x%02X), %u bytes",
           iohc_cmd::CMD_REMOVE_CONTROLLER, static_cast<unsigned>(frame_remove.size()));
  this->hub_->transmit_packet(frame_remove, static_cast<uint8_t>(this->repeat_count_));

  // Step 2: CMD_WRITE_PRIVATE (0x30) pushes the controller key. The key is
  // obfuscated with the public transfer key (keystream = AES(transfer_key, IV)
  // where IV is the controller node address repeated), then the on-air data is
  // enc_key(16) || manufacturer(0x02 = Somfy) || key-index(0x01). Only
  // cmd || enc_key (17 bytes) is authenticated by the MAC; the manufacturer
  // trailer is not.
  uint8_t key_data[18];
  iohc_proto::obfuscate_key_1w(aes128_ecb_encrypt, iohc_keys::TRANSFER_KEY, this->node_id_,
                               this->encryption_key_, key_data);
  key_data[16] = 0x02;  // manufacturer: Somfy
  key_data[17] = 0x01;  // key index
  auto frame_key =
      this->build_1w_frame(iohc_cmd::CMD_WRITE_PRIVATE, key_data, sizeof(key_data), iohc::BROADCAST_ADDR, 16);
  ESP_LOGD(TAG, "PROG: tx CMD_WRITE_PRIVATE (0x%02X), %u bytes (key omitted)",
           iohc_cmd::CMD_WRITE_PRIVATE, static_cast<unsigned>(frame_key.size()));
  this->hub_->transmit_packet(frame_key, static_cast<uint8_t>(this->repeat_count_));
  ESP_LOGI(TAG, "PROG: pairing frames sent");
}

// ---------------------------------------------------------------------------
// 1W Protocol (per-device: uses device key + rolling code)
// ---------------------------------------------------------------------------

void SomfyIohcCover::send_1w_command(uint16_t main_param) {
  // CMD_EXECUTE data: Originator(1) + ACEI(1) + MainParam(2) + FP1(1) + FP2(1).
  uint8_t data[6] = {
      iohc_cmd::ORIGINATOR_USER,
      iohc_cmd::ACEI_DEFAULT,
      static_cast<uint8_t>(main_param >> 8),
      static_cast<uint8_t>(main_param & 0xFF),
      0x00,  // FP1
      0x00,  // FP2
  };
  ESP_LOGD(TAG, "TX EXECUTE 1W: src=0x%06" PRIX32 " dst=BROADCAST mp=0x%04X", this->node_id_, main_param);
  auto frame = this->build_1w_frame(iohc_cmd::CMD_EXECUTE, data, sizeof(data), iohc::BROADCAST_ADDR);
  this->hub_->transmit_packet(frame, static_cast<uint8_t>(this->repeat_count_));
}

// ---------------------------------------------------------------------------
// 2W Protocol (challenge/response via hub session)
// ---------------------------------------------------------------------------

void SomfyIohcCover::send_2w_command(uint16_t main_param) {
  // Build CMD_EXECUTE data: Originator(1) + ACEI(1) + MainParam(2) + FP1(1) + FP2(1)
  uint8_t data[6] = {
      iohc_cmd::ORIGINATOR_USER,
      iohc_cmd::ACEI_2W,
      static_cast<uint8_t>(main_param >> 8),
      static_cast<uint8_t>(main_param & 0xFF),
      0x00,  // FP1
      0x00,  // FP2
  };

  this->hub_->send_2w_command(
      this->node_id_, this->target_node_, iohc_cmd::CMD_EXECUTE,
      data, sizeof(data), this->encryption_key_,
      [this](bool success, const IohcDecodedPacket *response) {
        this->on_2w_result_(success, response);
      });
}

void SomfyIohcCover::on_2w_result_(bool success, const IohcDecodedPacket *response) {
  if (success) {
    ESP_LOGD(TAG, "2W command acknowledged by 0x%06" PRIX32, this->target_node_);
    if (response && response->data_len > 0) {
      // Parse status byte if present (CMD 0xFE: first data byte is status code)
      if (response->cmd == 0xFE && response->data_len >= 1) {
        uint8_t status = response->data[0];
        if (status == 0x05) {
          ESP_LOGD(TAG, "2W: Actuator reports NO ERROR");
        } else {
          ESP_LOGW(TAG, "2W: Actuator reports error code 0x%02X", status);
        }
      }
    }
  } else {
    ESP_LOGW(TAG, "2W command to 0x%06" PRIX32 " failed (no response)", this->target_node_);
  }
}

std::vector<uint8_t> SomfyIohcCover::build_1w_frame(uint8_t cmd, const uint8_t *data,
                                                      size_t data_len, uint32_t dest_node, size_t auth_len) {
  if (auth_len == SIZE_MAX || auth_len > data_len)
    auth_len = data_len;

  const uint16_t sequence = this->storage_->nextCode();

  std::vector<uint8_t> frame;
  frame.reserve(2 + 6 + 1 + data_len + 2 + 6 + 2);

  // CtrlByte0 placeholder — the size field depends on the final body length and
  // is filled in once the body (ctrl1..MAC) is assembled.
  frame.push_back(0x00);
  // CtrlByte1: 1W frames carry no Start/End framing bits (0x00).
  frame.push_back(0x00);

  // Destination node ID (3 bytes, big-endian)
  frame.push_back(static_cast<uint8_t>((dest_node >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((dest_node >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(dest_node & 0xFF));

  // Source node ID (3 bytes, big-endian)
  frame.push_back(static_cast<uint8_t>((this->node_id_ >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((this->node_id_ >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(this->node_id_ & 0xFF));

  // Command
  frame.push_back(cmd);

  // Data
  for (size_t i = 0; i < data_len; i++) {
    frame.push_back(data[i]);
  }

  // Sequence (2 bytes, big-endian) — sits between the data and the MAC.
  frame.push_back(static_cast<uint8_t>(sequence >> 8));
  frame.push_back(static_cast<uint8_t>(sequence & 0xFF));

  // MAC (6 bytes) over the authenticated payload cmd || data[0..auth_len).
  uint8_t mac_payload[1 + 16];
  size_t mac_payload_len = 1 + auth_len;
  mac_payload[0] = cmd;
  memcpy(mac_payload + 1, data, auth_len);

  uint8_t iv[16];
  iohc_proto::build_iv_1w(mac_payload, mac_payload_len, sequence, iv);
  uint8_t mac[6];
  iohc_proto::compute_mac(aes128_ecb_encrypt, this->encryption_key_, iv, mac);
  for (int i = 0; i < 6; i++) {
    frame.push_back(mac[i]);
  }

  // CtrlByte0: order=11, isOneWay=1, size = body length (everything after ctrl0,
  // excluding the trailing CRC) masked to 5 bits.
  const size_t size_field = frame.size() - 1;
  if (size_field > 0x1F) {
    // io-homecontrol's 5-bit length field caps a single frame at 31 body bytes
    // (max ~21 data bytes). Larger frames must be fragmented; a receiver reads
    // length & 0x1F and parses the wrong length. Pairing's CMD_WRITE_PRIVATE
    // (0x30) is 34 bytes and trips this — flagging it so it shows in logs.
    ESP_LOGW(TAG, "1W frame cmd=0x%02X body=%u B exceeds 31-byte 1W limit; size field wraps to %u",
             cmd, static_cast<unsigned>(size_field), static_cast<unsigned>(size_field & 0x1F));
  }
  frame[0] = static_cast<uint8_t>(0xE0 | (size_field & 0x1F));

  // CRC-16-KERMIT
  uint16_t crc = crc16_kermit(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  ESP_LOGD(TAG, "1W frame cmd=0x%02X dst=0x%06" PRIX32 " seq=%u crc=0x%04X ctrl0=0x%02X (%u B)", cmd, dest_node,
           sequence, crc, frame[0], static_cast<unsigned>(frame.size()));
  return frame;
}

// ---------------------------------------------------------------------------
// RX callback from hub
// ---------------------------------------------------------------------------

void SomfyIohcCover::on_iohc_packet_(const IohcDecodedPacket &pkt) {
#ifdef USE_SOMFY_IOHC_RX
  // --- State-sync + discovery: surface movement commands sent by physical
  //     io-homecontrol remotes so the HA UI matches the motor when driven
  //     outside HA. Runs for both 1W and 2W covers — a foreign remote command
  //     is a 1W broadcast frame regardless of this cover's own mode, so we must
  //     inspect it before the 2W target-node filter below drops it. ---
  if (pkt.src_node != this->node_id_ && pkt.cmd == iohc_cmd::CMD_EXECUTE) {
    char hexbuf[RX_HEX_MAX_BYTES * 3 + 1];
    format_payload_hex(pkt.data, pkt.data_len, hexbuf, sizeof(hexbuf));
    ESP_LOGD(TAG, "RX EXECUTE src=0x%06" PRIX32 " dst=0x%06" PRIX32 " len=%u data=[%s] rssi=%.1f",
             pkt.src_node, pkt.dest_node, static_cast<unsigned>(pkt.data_len), hexbuf, pkt.rssi);

    uint16_t main_param;
    const bool decoded = decode_execute_param_(pkt, main_param);

    // Publish to the discovery text sensor regardless of allow-list so unknown
    // remote IDs (and unexpected payload layouts) can be learned, mirroring RTS.
    if (this->log_text_sensor_ != nullptr) {
      char buf[80];
      if (decoded)
        snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04X",
                 pkt.src_node, main_param_name(main_param), main_param);
      else
        snprintf(buf, sizeof(buf), "0x%06" PRIX32 " RAW [%s]", pkt.src_node, hexbuf);
      this->log_text_sensor_->publish_state(buf);
    }

    if (decoded && this->is_allowed_remote_(pkt.src_node)) {
      if (this->rx_is_duplicate_(pkt.src_node, main_param))
        return;  // repeat frame from the remote's burst — already handled
      ESP_LOGD(TAG, "RX sync: remote 0x%06" PRIX32 " %s (mp=0x%04X) rssi=%.1f",
               pkt.src_node, main_param_name(main_param), main_param, pkt.rssi);
      this->handle_rx_command_(main_param);
      return;
    }
  }
#endif  // USE_SOMFY_IOHC_RX

  // --- 2W feedback / addressed-packet logging (status replies, ACKs). ---
  if (pkt.dest_node != this->node_id_ && pkt.dest_node != iohc::BROADCAST_ADDR)
    return;

  // For 2W mode, also accept packets from our target actuator
  if (this->mode_ == IohcMode::MODE_2W && pkt.src_node != this->target_node_)
    return;

  ESP_LOGD(TAG, "RX for node 0x%06" PRIX32 ": src=0x%06" PRIX32 " cmd=0x%02X rssi=%.1f", this->node_id_,
           pkt.src_node, pkt.cmd, pkt.rssi);
}

#ifdef USE_SOMFY_IOHC_RX

bool SomfyIohcCover::is_allowed_remote_(uint32_t code) const {
  // Empty list = discovery / accept-all (matches RTS semantics).
  return this->receive_remote_codes_.empty() ||
         std::binary_search(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), code);
}

bool SomfyIohcCover::decode_execute_param_(const IohcDecodedPacket &pkt, uint16_t &main_param) {
  if (pkt.cmd != iohc_cmd::CMD_EXECUTE || pkt.data == nullptr)
    return false;
  // Standard io-homecontrol CMD_EXECUTE payload:
  //   Originator(1) ACEI(1) MainParameter(2) [FP1(1) FP2(1)] ...
  // The MainParameter sits at a fixed offset from the start of the data field,
  // ahead of any trailing HMAC/MAC bytes, so reading data[2..3] is robust to
  // the variable frame tail.
  if (pkt.data_len < 4)
    return false;
  main_param = (static_cast<uint16_t>(pkt.data[2]) << 8) | pkt.data[3];
  return true;
}

bool SomfyIohcCover::rx_is_duplicate_(uint32_t src, uint16_t main_param) {
  const uint32_t now = millis();
  if (this->rx_dedup_valid_ && src == this->rx_dedup_src_ && main_param == this->rx_dedup_param_ &&
      (now - this->rx_dedup_ms_) < RX_DEDUP_WINDOW_MS) {
    this->rx_dedup_ms_ = now;  // extend the window across the whole burst
    return true;
  }
  this->rx_dedup_valid_ = true;
  this->rx_dedup_src_ = src;
  this->rx_dedup_param_ = main_param;
  this->rx_dedup_ms_ = now;
  return false;
}

void SomfyIohcCover::handle_rx_command_(uint16_t main_param) {
  switch (main_param) {
    case iohc_cmd::MP_OPEN:
      this->start_rx_sync(cover::COVER_OPERATION_OPENING);
      break;
    case iohc_cmd::MP_CLOSE:
      this->start_rx_sync(cover::COVER_OPERATION_CLOSING);
      break;
    case iohc_cmd::MP_STOP:
    case iohc_cmd::MP_MY:
      this->stop_rx_sync();
      break;
    default:
      // Unknown / position command: leave UI untouched (discovery already logged).
      break;
  }
}

void SomfyIohcCover::start_rx_sync(cover::CoverOperation op) {
  this->rx_sync_active_ = true;
  this->rx_operation_ = op;
  this->rx_start_ms_ = millis();
  this->rx_start_pos_ = this->position;
  this->rx_last_publish_ms_ = 0;
  this->rx_last_published_pos_ = -1.0f;
  this->current_operation = op;
  this->publish_state();
}

void SomfyIohcCover::stop_rx_sync() {
  this->rx_sync_active_ = false;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
}

void SomfyIohcCover::loop() {
  if (this->rx_sync_active_) {
    const uint32_t now_ms = millis();
    const bool opening = this->rx_operation_ == cover::COVER_OPERATION_OPENING;

    const uint32_t full_dur_ms = opening ? this->open_duration_ : this->close_duration_;
    const float target = opening ? cover::COVER_OPEN : cover::COVER_CLOSED;

    // Only the remaining travel takes time, so a half-open cover reaches the
    // end stop in half the configured duration.
    float remaining = std::fabs(target - this->rx_start_pos_);
    if (remaining < 0.0f)
      remaining = 0.0f;
    if (remaining > 1.0f)
      remaining = 1.0f;

    const uint32_t dur_ms = static_cast<uint32_t>(static_cast<float>(full_dur_ms) * remaining);
    if (dur_ms == 0) {
      this->position = target;
      this->rx_last_published_pos_ = this->position;
      this->stop_rx_sync();
      return;
    }

    const uint32_t elapsed = now_ms - this->rx_start_ms_;
    const float progress = (elapsed >= dur_ms) ? 1.0f : (static_cast<float>(elapsed) / static_cast<float>(dur_ms));

    float new_pos = this->rx_start_pos_ + (target - this->rx_start_pos_) * progress;
    if (new_pos < cover::COVER_CLOSED)
      new_pos = cover::COVER_CLOSED;
    if (new_pos > cover::COVER_OPEN)
      new_pos = cover::COVER_OPEN;
    this->position = new_pos;

    const bool time_ok =
        (this->rx_last_publish_ms_ == 0) || ((now_ms - this->rx_last_publish_ms_) >= RX_SYNC_PUBLISH_INTERVAL_MS);
    const bool delta_ok =
        (this->rx_last_published_pos_ < 0.0f) || (std::fabs(this->position - this->rx_last_published_pos_) >= 0.01f);
    if (time_ok && delta_ok) {
      this->rx_last_publish_ms_ = now_ms;
      this->rx_last_published_pos_ = this->position;
      this->publish_state();
    }

    if (progress >= 1.0f) {
      this->rx_last_published_pos_ = this->position;
      this->stop_rx_sync();
    }
    return;
  }

  SomfyTimeBasedCover::loop();
}

#endif  // USE_SOMFY_IOHC_RX

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
