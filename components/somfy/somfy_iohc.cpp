#include "somfy_iohc.h"

#ifdef USE_SOMFY_IOHC

#include "iohc_protocol.h"
#include "esphome/core/log.h"
#include <cinttypes>
#include <cmath>
#include <cstring>

#ifdef USE_SOMFY_IOHC_RX
#include "esphome/components/text_sensor/text_sensor.h"
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

const char *button_action_name(uint8_t action) {
  switch (action) {
    case iohc_cmd::BUTTON_ACTION_OPEN: return "OPEN";
    case iohc_cmd::BUTTON_ACTION_CLOSE: return "CLOSE";
    case iohc_cmd::BUTTON_ACTION_STOP_MY: return "STOP/MY";
    case iohc_cmd::BUTTON_ACTION_TILT_CLOCKWISE: return "TILT CLOCKWISE";
    case iohc_cmd::BUTTON_ACTION_TILT_COUNTERCLOCKWISE: return "TILT COUNTERCLOCKWISE";
    default: return "BUTTON";
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

void SomfyIohcCover::set_encryption_key(const uint8_t key[16]) {
  if (key == nullptr) {
    ESP_LOGE(TAG, "Encryption key is missing");
    return;
  }
  memcpy(this->encryption_key_, key, sizeof(this->encryption_key_));
  this->has_custom_key_ = true;
}

void SomfyIohcCover::set_venetian(bool enabled, uint8_t tilt_steps,
                                  bool tilt_inverted, uint8_t my_tilt_step) {
  this->venetian_ = enabled;
  this->tilt_steps_ = clamp<uint8_t>(tilt_steps, 1, 254);
  this->tilt_inverted_ = tilt_inverted;
  this->my_tilt_step_ = std::min<uint8_t>(my_tilt_step, this->tilt_steps_);
  if (!enabled)
    this->cancel_1w_tilt_sequence();
}

cover::CoverTraits SomfyIohcCover::get_traits() {
  auto traits = SomfyTimeBasedCover::get_traits();
  traits.set_supports_tilt(this->venetian_);
  return traits;
}

void SomfyIohcCover::reconfigure_storage(const char *ns, const char *key, uint16_t initial_code) {
  this->storage_namespace_ = ns;
  this->storage_key_ = key;
  this->initial_rolling_code_ = initial_code == 0 ? 1 : initial_code;
  // This is also safe after setup: NVSRollingCodeStorage closes its old handle
  // before the replacement stream is opened.
  if (this->storage_ != nullptr) {
    this->storage_ = std::make_unique<NVSRollingCodeStorage>(
        this->storage_namespace_, this->storage_key_, this->initial_rolling_code_);
  }
}

uint16_t SomfyIohcCover::peek_next_rolling_code() const {
  return this->storage_ == nullptr ? 0 : this->storage_->peekNextCode();
}

void SomfyIohcCover::runtime_clear_receive_remote_codes() {
#ifdef USE_SOMFY_IOHC_RX
  this->receive_remote_codes_.clear();
#endif
}

bool SomfyIohcCover::runtime_program() { return this->program(); }

void SomfyIohcCover::runtime_open() {
  auto call = this->make_call();
  call.set_command_open();
  call.perform();
}

void SomfyIohcCover::runtime_close() {
  auto call = this->make_call();
  call.set_command_close();
  call.perform();
}

void SomfyIohcCover::runtime_stop() {
  auto call = this->make_call();
  call.set_command_stop();
  call.perform();
}

void SomfyIohcCover::runtime_my() { this->my(); }

void SomfyIohcCover::runtime_set_position(float position) {
  auto call = this->make_call();
  call.set_position(clamp(position, 0.0f, 1.0f));
  call.perform();
}

void SomfyIohcCover::runtime_set_tilt(float tilt) {
  auto call = this->make_call();
  call.set_tilt(clamp(tilt, 0.0f, 1.0f));
  call.perform();
}

void SomfyIohcCover::runtime_tilt_step(bool clockwise) {
  if (!this->runtime_enabled_ || !this->venetian_) {
    ESP_LOGW(TAG, "Ignoring tilt step for a non-Venetian or unused slot");
    return;
  }
  const int8_t logical_direction = (clockwise != this->tilt_inverted_) ? 1 : -1;
  const float target = clamp(this->tilt + logical_direction / static_cast<float>(this->tilt_steps_),
                             0.0f, 1.0f);
  this->start_tilt_steps_(1, logical_direction, target);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void SomfyIohcCover::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Somfy iohc cover...");

  if (this->storage_namespace_ == nullptr || this->storage_key_ == nullptr) {
    ESP_LOGE(TAG, "Rolling-code storage identity is missing; RF transmission will remain disabled");
    this->runtime_enabled_ = false;
  } else {
    this->storage_ = std::make_unique<NVSRollingCodeStorage>(
        this->storage_namespace_, this->storage_key_, this->initial_rolling_code_);
  }
  const uint16_t next_code = this->storage_ == nullptr ? 0 : this->storage_->peekNextCode();
  if (next_code == 0) {
    ESP_LOGE(TAG, "Rolling-code storage is unavailable; RF transmission will remain disabled");
  } else {
    ESP_LOGCONFIG(TAG, "Next rolling code: %u", next_code);
  }

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

  if (this->prog_button_ != nullptr)
    this->prog_button_->add_on_press_callback([this]() { this->program(); });
  if (this->my_button_ != nullptr)
    this->my_button_->add_on_press_callback([this]() { this->my(); });

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
  ESP_LOGCONFIG(TAG, "  Missing-NVS initial rolling code: %u", this->initial_rolling_code_);
  ESP_LOGCONFIG(TAG, "  Custom key: %s", this->has_custom_key_ ? "yes" : "no (transfer key)");
  if (this->has_my_position_) {
    ESP_LOGCONFIG(TAG, "  MY position estimate: %.0f%%", this->my_position_ * 100.0f);
  }
  ESP_LOGCONFIG(TAG, "  Venetian tilt: %s", this->venetian_ ? "enabled" : "disabled");
  if (this->venetian_) {
    ESP_LOGCONFIG(TAG, "  Tilt calibration: %u steps, clockwise %s value",
                  static_cast<unsigned>(this->tilt_steps_),
                  this->tilt_inverted_ ? "decreases" : "increases");
  }
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
  this->cancel_1w_my_sequence();
  this->cancel_1w_tilt_sequence();
  ESP_LOGD(TAG, "OPEN node=0x%06" PRIX32 " mode=%s", this->node_id_,
           this->mode_ == IohcMode::MODE_2W ? "2W" : "1W");
  if (this->mode_ == IohcMode::MODE_2W) {
    this->send_2w_command(iohc_cmd::MP_OPEN);
  } else if (this->send_1w_command(iohc_cmd::MP_OPEN)) {
    this->set_lift_tilt_(true);
  }
}

void SomfyIohcCover::close() {
  this->cancel_1w_my_sequence();
  this->cancel_1w_tilt_sequence();
  ESP_LOGD(TAG, "CLOSE node=0x%06" PRIX32 " mode=%s", this->node_id_,
           this->mode_ == IohcMode::MODE_2W ? "2W" : "1W");
  if (this->mode_ == IohcMode::MODE_2W) {
    this->send_2w_command(iohc_cmd::MP_CLOSE);
  } else if (this->send_1w_command(iohc_cmd::MP_CLOSE)) {
    this->set_lift_tilt_(false);
  }
}

void SomfyIohcCover::stop() {
  this->cancel_1w_my_sequence();
  this->cancel_1w_tilt_sequence();
  ESP_LOGD(TAG, "STOP node=0x%06" PRIX32 " mode=%s", this->node_id_,
           this->mode_ == IohcMode::MODE_2W ? "2W" : "1W");
  if (this->mode_ == IohcMode::MODE_2W)
    this->send_2w_command(iohc_cmd::MP_STOP);
  else
    this->send_1w_command(iohc_cmd::MP_STOP);
}

void SomfyIohcCover::my() {
  // RF semantics do not depend on our time-based position estimate. A
  // dedicated MY action first sends stop-only D200, then emits the full native
  // MY sequence after the motor has settled. It therefore always means "go to
  // favourite" whether the motor was moving or idle. Cover STOP remains one
  // D200 frame and never recalls MY.
#ifdef USE_SOMFY_IOHC_RX
  if (this->rx_sync_.active())
    this->stop_rx_sync();
#endif
  this->cancel_1w_tilt_sequence();

  if (this->current_operation != cover::COVER_OPERATION_IDLE) {
    // Update the estimator without firing its normal STOP trigger: the MY
    // transmission below begins with the required stop-only D200.
    this->recompute_position_();
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->stop_prev_trigger_();
    this->publish_state();
  }

  ESP_LOGD(TAG, "MY node=0x%06" PRIX32 " target=%.0f%%", this->node_id_, this->my_position_ * 100.0f);
  if (this->mode_ == IohcMode::MODE_1W) {
    this->send_1w_my_sequence();
  } else {
    this->stop();
  }
}

void SomfyIohcCover::control(const cover::CoverCall &call) {
  if (!this->runtime_enabled_) {
    ESP_LOGW(TAG, "Ignoring command for an unused managed shutter slot");
    return;
  }
#ifdef USE_SOMFY_IOHC_RX
  // A Home Assistant command supersedes passive physical-remote/MY animation.
  const bool was_external_animation = this->rx_sync_.active();
  if (was_external_animation) {
    this->rx_sync_.stop();
    this->current_operation = cover::COVER_OPERATION_IDLE;
  }

  // Selecting the configured MY percentage while idle uses the motor's native
  // favourite rather than approximating it with OPEN/CLOSE followed by STOP.
  const auto requested_position = call.get_position();
  if (!call.get_stop() && !call.get_toggle().has_value() && requested_position.has_value() &&
      this->has_my_position_ &&
      std::fabs(*requested_position - this->my_position_) <= 0.005f &&
      !was_external_animation &&
      this->current_operation == cover::COVER_OPERATION_IDLE) {
    this->my();
    return;
  }
#endif

  const auto requested_tilt = call.get_tilt();
  if (requested_tilt.has_value() && this->venetian_)
    this->set_tilt_target_(*requested_tilt);

  SomfyTimeBasedCover::control(call);
}

bool SomfyIohcCover::program() {
  if (!this->runtime_enabled_ || this->hub_ == nullptr || this->storage_ == nullptr) {
    ESP_LOGE(TAG, "PROG refused: controller identity/storage is not active");
    return false;
  }
  ESP_LOGI(TAG, "PROG (pair): node=0x%06" PRIX32 " -> dest=BROADCAST(0x%06" PRIX32 ") repeat=%u", this->node_id_,
           static_cast<uint32_t>(iohc::BROADCAST_ADDR), iohc_cmd::PAIR_REPEAT_COUNT);

  // Step 1: CMD_REMOVE_CONTROLLER (0x39) carries a single data byte (0x00).
  uint8_t remove_data[1] = {0x00};
  auto frame_remove =
      this->build_1w_frame(iohc_cmd::CMD_REMOVE_CONTROLLER, remove_data, sizeof(remove_data), iohc::BROADCAST_ADDR);
  if (frame_remove.empty()) {
    ESP_LOGE(TAG, "PROG aborted: could not build remove-controller frame");
    return false;
  }
  ESP_LOGD(TAG, "PROG: tx CMD_REMOVE_CONTROLLER (0x%02X), %u bytes",
           iohc_cmd::CMD_REMOVE_CONTROLLER, static_cast<unsigned>(frame_remove.size()));
  this->hub_->transmit_packet(frame_remove, iohc_cmd::PAIR_REPEAT_COUNT);

  // Hardware-validated emitters leave a short gap between the announce/remove
  // burst and the install-key burst.
  delay(40);

  // Step 2: CMD_WRITE_PRIVATE (0x30) pushes the controller key. The key is
  // obfuscated with the public transfer key (keystream = AES(transfer_key, IV)
  // where IV is the controller node address repeated), then the on-air data is
  // enc_key(16) || manufacturer(0x02 = Somfy) || key-index(0x01). Real
  // controller captures and the hardware-validated CC1101 implementation use
  // the 31-byte management form: this command carries no six-byte 1W MAC.
  uint8_t encrypted_key[16];
  iohc_proto::obfuscate_key_1w(aes128_ecb_encrypt, iohc_keys::TRANSFER_KEY, this->node_id_,
                               this->encryption_key_, encrypted_key);
  const uint16_t key_sequence = this->next_rolling_code_();
  if (key_sequence == 0) {
    ESP_LOGE(TAG, "PROG aborted: rolling-code storage unavailable or exhausted");
    return false;
  }
  std::vector<uint8_t> frame_key;
  if (!iohc_proto::build_key_transfer_frame_1w(this->node_id_, iohc::BROADCAST_ADDR,
                                                encrypted_key, 0x02, 0x01,
                                                key_sequence, 0x00, frame_key)) {
    ESP_LOGE(TAG, "PROG aborted: could not build key-transfer frame");
    return false;
  }
  const uint16_t key_crc = static_cast<uint16_t>(frame_key[frame_key.size() - 2]) |
                           (static_cast<uint16_t>(frame_key[frame_key.size() - 1]) << 8);
  ESP_LOGD(TAG, "PROG: tx CMD_WRITE_PRIVATE (0x%02X), seq=%u crc=0x%04X, %u bytes (key omitted)",
           iohc_cmd::CMD_WRITE_PRIVATE, key_sequence, key_crc,
           static_cast<unsigned>(frame_key.size()));
  this->hub_->transmit_packet(frame_key, iohc_cmd::PAIR_REPEAT_COUNT);
  ESP_LOGI(TAG, "PROG: pairing frames sent");
  return true;
}

// ---------------------------------------------------------------------------
// 1W Protocol (per-device: uses device key + rolling code)
// ---------------------------------------------------------------------------

bool SomfyIohcCover::send_1w_command(uint16_t main_param) {
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
  if (frame.empty()) {
    ESP_LOGE(TAG, "TX EXECUTE 1W aborted: frame construction failed");
    return false;
  }
  // Six copies improved acceptance on the hardware-validated link. Keep this
  // configurable per shutter for unusually strong/weak RF paths; pairing uses
  // the separate four-copy remote pattern above.
  this->hub_->transmit_packet(frame, static_cast<uint8_t>(this->repeat_count_));
  return true;
}

bool SomfyIohcCover::send_1w_button_event(uint8_t action, bool released) {
  // Exact clear payloads captured from the paired Situo remote:
  //   press:   02 FF 01 43 <action> 0C 00 00
  //   release: 02 FF 01 43 <action> 05 FF 00
  // STOP/MY uses action 0x02. Each frame has its own persisted sequence and MAC.
  uint8_t data[8] = {
      0x02,
      0xFF,
      iohc_cmd::ORIGINATOR_USER,
      iohc_cmd::ACEI_DEFAULT,
      action,
      static_cast<uint8_t>(released ? 0x05 : 0x0C),
      static_cast<uint8_t>(released ? 0xFF : 0x00),
      0x00,
  };
  ESP_LOGD(TAG, "TX BUTTON EVENT 1W: src=0x%06" PRIX32 " action=0x%02X phase=%s", this->node_id_, action,
           released ? "release" : "press");
  auto frame = this->build_1w_frame(iohc_cmd::CMD_BUTTON_EVENT, data, sizeof(data), iohc::BROADCAST_ADDR);
  if (frame.empty()) {
    ESP_LOGE(TAG, "TX BUTTON EVENT 1W aborted: frame construction failed");
    return false;
  }
  this->hub_->transmit_packet(frame, static_cast<uint8_t>(this->repeat_count_));
  return true;
}

bool SomfyIohcCover::send_1w_my_execute() {
  // Exact extended EXECUTE payload captured from a Situo 5 Variation MY press:
  //   01 43 D2 00 20 D2 00 00
  // This is deliberately distinct from the ordinary six-byte D200 STOP frame.
  const uint8_t data[8] = {
      iohc_cmd::ORIGINATOR_USER,
      iohc_cmd::ACEI_DEFAULT,
      static_cast<uint8_t>(iohc_cmd::MP_STOP >> 8),
      static_cast<uint8_t>(iohc_cmd::MP_STOP & 0xFF),
      0x20,
      0xD2,
      0x00,
      0x00,
  };
  ESP_LOGD(TAG, "TX MY EXECUTE 1W: src=0x%06" PRIX32, this->node_id_);
  auto frame = this->build_1w_frame(iohc_cmd::CMD_EXECUTE, data, sizeof(data),
                                    iohc::BROADCAST_ADDR);
  if (frame.empty()) {
    ESP_LOGE(TAG, "TX MY EXECUTE 1W aborted: frame construction failed");
    return false;
  }
  this->hub_->transmit_packet(frame, static_cast<uint8_t>(this->repeat_count_));
  return true;
}

bool SomfyIohcCover::send_1w_tilt_execute(bool clockwise) {
  // Exact clear EXECUTE payloads captured from the Situo Variation wheel:
  //   clockwise:        01 43 D2 00 20 CD 2E 00
  //   counterclockwise: 01 43 D2 00 20 CC A2 00
  // The following private 0x20 action carries the authoritative direction,
  // but the motor expects this complete leading frame as part of the gesture.
  uint8_t data[8] = {
      iohc_cmd::ORIGINATOR_USER,
      iohc_cmd::ACEI_DEFAULT,
      static_cast<uint8_t>(iohc_cmd::MP_STOP >> 8),
      static_cast<uint8_t>(iohc_cmd::MP_STOP & 0xFF),
      0x20,
      static_cast<uint8_t>(clockwise ? 0xCD : 0xCC),
      static_cast<uint8_t>(clockwise ? 0x2E : 0xA2),
      0x00,
  };
  ESP_LOGD(TAG, "TX TILT EXECUTE 1W: src=0x%06" PRIX32 " direction=%s",
           this->node_id_, clockwise ? "clockwise" : "counterclockwise");
  auto frame = this->build_1w_frame(iohc_cmd::CMD_EXECUTE, data, sizeof(data),
                                    iohc::BROADCAST_ADDR);
  if (frame.empty()) {
    ESP_LOGE(TAG, "TX TILT EXECUTE 1W aborted: frame construction failed");
    return false;
  }
  this->hub_->transmit_packet(frame, static_cast<uint8_t>(this->repeat_count_));
  return true;
}

float SomfyIohcCover::logical_tilt_for_physical_step_(
    uint8_t physical_step) const {
  const float physical = std::min<uint8_t>(physical_step, this->tilt_steps_) /
                         static_cast<float>(this->tilt_steps_);
  return this->tilt_inverted_ ? 1.0f - physical : physical;
}

void SomfyIohcCover::set_lift_tilt_(bool opening, bool publish) {
  if (!this->venetian_)
    return;
  // Home Assistant defines 100% tilt as fully open and 0% as fully closed.
  // The tested motor establishes those absolute slat states while raising and
  // lowering respectively. STOP leaves the mechanically established angle in
  // place, so the published estimate must remain at that endpoint.
  this->tilt = opening ? 1.0f : 0.0f;
  if (publish)
    this->publish_state();
}

void SomfyIohcCover::set_my_tilt_(bool publish) {
  if (!this->venetian_)
    return;
  this->tilt = this->logical_tilt_for_physical_step_(this->my_tilt_step_);
  if (publish)
    this->publish_state();
}

void SomfyIohcCover::set_tilt_target_(float target) {
  if (!this->runtime_enabled_ || !this->venetian_) {
    ESP_LOGW(TAG, "Ignoring tilt target for a non-Venetian or unused slot");
    return;
  }
  target = clamp(target, 0.0f, 1.0f);
  // An endpoint request is also the absolute-position resynchronisation path.
  // Always sweep beyond a complete calibrated range so the first command
  // after pairing/reboot is correct even if the restored estimate and physical
  // slats disagree. The motor ignores the deliberate extra endpoint detents.
  const bool endpoint = target == 0.0f || target == 1.0f;
  if (endpoint) {
    const int16_t steps = static_cast<int16_t>(this->tilt_steps_) +
                          iohc_cmd::TILT_ENDPOINT_MARGIN_STEPS;
    this->start_tilt_steps_(steps, target == 1.0f ? 1 : -1, target);
    return;
  }

  // A 1W Venetian motor accepts relative detents, not arbitrary percentages.
  // Quantize both ends to the calibrated step grid, transmit exactly their
  // difference, and publish the reachable target rather than the user's raw
  // request when the sequence completes.
  const int16_t current_step = clamp<int16_t>(
      static_cast<int16_t>(std::lround(this->tilt * this->tilt_steps_)), 0,
      this->tilt_steps_);
  const int16_t target_step = clamp<int16_t>(
      static_cast<int16_t>(std::lround(target * this->tilt_steps_)), 0,
      this->tilt_steps_);
  const float reachable_target =
      target_step / static_cast<float>(this->tilt_steps_);
  const int16_t step_delta = target_step - current_step;
  const int16_t steps = std::abs(step_delta);
  if (steps == 0) {
    this->tilt = reachable_target;
    this->publish_state();
    return;
  }
  this->start_tilt_steps_(steps, step_delta > 0 ? 1 : -1,
                          reachable_target);
}

void SomfyIohcCover::start_tilt_steps_(int16_t steps, int8_t logical_direction,
                                       float target) {
  this->cancel_1w_my_sequence();
  this->cancel_1w_tilt_sequence();

#ifdef USE_SOMFY_IOHC_RX
  if (this->rx_sync_.active())
    this->stop_rx_sync();
#endif
  if (this->current_operation != cover::COVER_OPERATION_IDLE) {
    // The first tilt EXECUTE is D200 and therefore stops the lift motor. Keep
    // the time-based lift estimate accurate without emitting a separate STOP.
    this->recompute_position_();
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->stop_prev_trigger_();
    this->publish_state();
  }

  this->tilt_steps_remaining_ = std::max<int16_t>(1, steps);
  this->tilt_direction_ = logical_direction >= 0 ? 1 : -1;
  this->tilt_target_ = clamp(target, 0.0f, 1.0f);
  this->send_next_tilt_step_();
}

void SomfyIohcCover::send_next_tilt_step_() {
  if (this->tilt_steps_remaining_ <= 0 || !this->runtime_enabled_ || !this->venetian_)
    return;

  const bool clockwise = (this->tilt_direction_ > 0) != this->tilt_inverted_;
  if (!this->send_1w_tilt_execute(clockwise)) {
    this->cancel_1w_tilt_sequence();
    return;
  }

  this->set_timeout("iohc-tilt-event", iohc_cmd::TILT_EVENT_DELAY_MS,
                    [this, clockwise]() {
    const uint8_t action = clockwise ? iohc_cmd::BUTTON_ACTION_TILT_CLOCKWISE
                                     : iohc_cmd::BUTTON_ACTION_TILT_COUNTERCLOCKWISE;
    if (!this->send_1w_button_event(action, false)) {
      this->cancel_1w_tilt_sequence();
      return;
    }

    this->tilt_steps_remaining_--;
    if (this->tilt_steps_remaining_ <= 0) {
      this->tilt = this->tilt_target_;
      this->tilt_direction_ = 0;
      this->publish_state();
      return;
    }

    this->tilt = clamp(this->tilt + this->tilt_direction_ /
                                      static_cast<float>(this->tilt_steps_),
                       0.0f, 1.0f);
    this->publish_state();
    this->set_timeout("iohc-tilt-next", iohc_cmd::TILT_NEXT_STEP_DELAY_MS,
                      [this]() { this->send_next_tilt_step_(); });
  });
}

void SomfyIohcCover::cancel_1w_tilt_sequence() {
  this->cancel_timeout("iohc-tilt-event");
  this->cancel_timeout("iohc-tilt-next");
  this->tilt_steps_remaining_ = 0;
  this->tilt_direction_ = 0;
}

void SomfyIohcCover::cancel_1w_my_sequence() {
  this->cancel_timeout("iohc-my-recall");
  this->cancel_timeout("iohc-my-press");
  this->cancel_timeout("iohc-my-release");
}

bool SomfyIohcCover::send_1w_my_sequence() {
  this->cancel_1w_my_sequence();
  // D200 by itself is a safe, stateless pre-stop: it stops a moving motor and
  // does nothing while idle. Once the motor has settled, reproduce a complete
  // native MY press. Sending two complete MY presses would be wrong from idle,
  // because the second one would stop the recall started by the first.
  if (!this->send_1w_command(iohc_cmd::MP_STOP))
    return false;

  this->set_timeout("iohc-my-recall", iohc_cmd::MY_PRESTOP_SETTLE_MS, [this]() {
    if (!this->send_1w_my_execute())
      return;
    this->set_timeout("iohc-my-press", iohc_cmd::MY_EVENT_DELAY_MS, [this]() {
      if (!this->send_1w_button_event(iohc_cmd::BUTTON_ACTION_STOP_MY, false))
        return;
#ifdef USE_SOMFY_IOHC_RX
      if (this->has_my_position_)
        this->start_rx_sync_to(this->my_position_);
#endif
      this->set_timeout("iohc-my-release", iohc_cmd::MY_RELEASE_DELAY_MS, [this]() {
        this->send_1w_button_event(iohc_cmd::BUTTON_ACTION_STOP_MY, true);
      });
    });
  });
  return true;
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

  const uint16_t sequence = this->next_rolling_code_();
  if (sequence == 0) {
    ESP_LOGE(TAG, "Cannot build 1W frame cmd=0x%02X: rolling-code storage unavailable or exhausted", cmd);
    return {};
  }

  std::vector<uint8_t> frame;
  if (!iohc_proto::build_frame_1w(aes128_ecb_encrypt, this->encryption_key_, this->node_id_, dest_node,
                                  cmd, data, data_len, auth_len, sequence, frame)) {
    ESP_LOGE(TAG, "Cannot build 1W frame cmd=0x%02X: invalid data or size", cmd);
    return {};
  }

  const uint16_t crc = static_cast<uint16_t>(frame[frame.size() - 2]) |
                       (static_cast<uint16_t>(frame[frame.size() - 1]) << 8);
  ESP_LOGD(TAG, "1W frame cmd=0x%02X dst=0x%06" PRIX32 " seq=%u crc=0x%04X ctrl0=0x%02X (%u B)", cmd, dest_node,
           sequence, crc, frame[0], static_cast<unsigned>(frame.size()));
  return frame;
}

uint16_t SomfyIohcCover::next_rolling_code_() {
  if (this->storage_ == nullptr)
    return 0;
  const uint16_t sequence = this->storage_->nextCode();
  if (sequence != 0 && this->rolling_code_callback_)
    this->rolling_code_callback_(static_cast<uint16_t>(sequence + 1));
  return sequence;
}

// ---------------------------------------------------------------------------
// RX callback from hub
// ---------------------------------------------------------------------------

void SomfyIohcCover::on_iohc_packet_(const IohcDecodedPacket &pkt) {
  if (!this->runtime_enabled_)
    return;

#ifdef USE_SOMFY_IOHC_RX
  // --- State-sync + discovery: surface movement commands sent by physical
  //     io-homecontrol remotes so the HA UI matches the motor when driven
  //     outside HA. Runs for both 1W and 2W covers — a foreign remote command
  //     is a 1W broadcast frame regardless of this cover's own mode, so we must
  //     inspect it before the 2W target-node filter below drops it. ---
  if (this->venetian_ && pkt.src_node != this->node_id_ &&
      pkt.cmd == iohc_cmd::CMD_BUTTON_EVENT) {
    uint8_t action = 0;
    if (decode_button_action_(pkt, action) &&
        (action == iohc_cmd::BUTTON_ACTION_TILT_CLOCKWISE ||
         action == iohc_cmd::BUTTON_ACTION_TILT_COUNTERCLOCKWISE)) {
      uint16_t sequence = 0;
      const bool has_sequence = iohc_proto::extract_sequence_1w(
          pkt.data, pkt.data_len, sequence);
      const uint16_t event_code = static_cast<uint16_t>(0xF000U | action);
      const bool duplicate = this->rx_deduplicator_.is_duplicate(
          millis(), pkt.src_node, event_code, sequence, has_sequence,
          RX_DEDUP_WINDOW_MS);
      if (!duplicate && this->log_text_sensor_ != nullptr) {
        char buf[112];
        this->rx_event_counter_++;
        if (has_sequence)
          snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s seq=0x%04X event=%" PRIu32,
                   pkt.src_node, button_action_name(action), sequence,
                   this->rx_event_counter_);
        else
          snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s event=%" PRIu32,
                   pkt.src_node, button_action_name(action), this->rx_event_counter_);
        this->log_text_sensor_->publish_state(buf);
      }
      if (this->is_allowed_remote_(pkt.src_node)) {
        if (duplicate)
          return;
        if (this->remote_command_callback_)
          this->remote_command_callback_(event_code, pkt.src_node, pkt.rssi);
        this->handle_rx_tilt_step_(
            action == iohc_cmd::BUTTON_ACTION_TILT_CLOCKWISE);
        return;
      }
    }
  }

  if (pkt.src_node != this->node_id_ && pkt.cmd == iohc_cmd::CMD_EXECUTE) {
    char hexbuf[RX_HEX_MAX_BYTES * 3 + 1];
    format_payload_hex(pkt.data, pkt.data_len, hexbuf, sizeof(hexbuf));
    ESP_LOGD(TAG, "RX EXECUTE src=0x%06" PRIX32 " dst=0x%06" PRIX32 " len=%u data=[%s] rssi=%.1f",
             pkt.src_node, pkt.dest_node, static_cast<unsigned>(pkt.data_len), hexbuf, pkt.rssi);

    uint16_t main_param;
    const bool decoded = decode_execute_param_(pkt, main_param);
    uint16_t sequence = 0;
    const bool has_sequence = decoded && iohc_proto::extract_sequence_1w(pkt.data, pkt.data_len, sequence);
    const bool tilt_execute = decoded && this->venetian_ && is_tilt_execute_(pkt);
    const bool duplicate = decoded && this->rx_deduplicator_.is_duplicate(
                                          millis(), pkt.src_node, main_param, sequence,
                                          has_sequence, RX_DEDUP_WINDOW_MS);

    // Publish to the discovery text sensor regardless of allow-list so unknown
    // remote IDs can be learned. Sequence-aware burst deduplication publishes
    // one update per physical press, while the sequence and event number force
    // repeated presses of the same button to remain visible in HA history.
    if (!tilt_execute && this->log_text_sensor_ != nullptr && (!decoded || !duplicate)) {
      char buf[112];
      if (decoded) {
        this->rx_event_counter_++;
        if (has_sequence)
          snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04X seq=0x%04X event=%" PRIu32,
                   pkt.src_node, main_param_name(main_param), main_param, sequence, this->rx_event_counter_);
        else
          snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04X event=%" PRIu32,
                   pkt.src_node, main_param_name(main_param), main_param, this->rx_event_counter_);
      } else {
        snprintf(buf, sizeof(buf), "0x%06" PRIX32 " RAW [%s]", pkt.src_node, hexbuf);
      }
      this->log_text_sensor_->publish_state(buf);
    }

    if (decoded && this->is_allowed_remote_(pkt.src_node)) {
      if (duplicate)
        return;  // repeat frame from the remote's burst — already handled
      if (tilt_execute)
        return;  // the following private 0x20 action supplies the direction
      ESP_LOGD(TAG, "RX sync: remote 0x%06" PRIX32 " %s (mp=0x%04X) rssi=%.1f",
               pkt.src_node, main_param_name(main_param), main_param, pkt.rssi);
      if (this->remote_command_callback_)
        this->remote_command_callback_(main_param, pkt.src_node, pkt.rssi);
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

bool SomfyIohcCover::decode_button_action_(const IohcDecodedPacket &pkt, uint8_t &action) {
  if (pkt.cmd != iohc_cmd::CMD_BUTTON_EVENT || pkt.data == nullptr || pkt.data_len < 6)
    return false;
  // Captured private payload: 02 FF 01 43 <action> <phase> ...
  if (pkt.data[0] != 0x02 || pkt.data[1] != 0xFF ||
      pkt.data[2] != iohc_cmd::ORIGINATOR_USER ||
      pkt.data[3] != iohc_cmd::ACEI_DEFAULT)
    return false;
  action = pkt.data[4];
  return true;
}

bool SomfyIohcCover::is_tilt_execute_(const IohcDecodedPacket &pkt) {
  if (pkt.cmd != iohc_cmd::CMD_EXECUTE || pkt.data == nullptr || pkt.data_len < 8)
    return false;
  return pkt.data[2] == static_cast<uint8_t>(iohc_cmd::MP_STOP >> 8) &&
         pkt.data[3] == static_cast<uint8_t>(iohc_cmd::MP_STOP & 0xFF) &&
         pkt.data[4] == 0x20 && pkt.data[7] == 0x00 &&
         ((pkt.data[5] == 0xCD && pkt.data[6] == 0x2E) ||
          (pkt.data[5] == 0xCC && pkt.data[6] == 0xA2));
}

void SomfyIohcCover::handle_rx_command_(uint16_t main_param) {
  switch (main_param) {
    case iohc_cmd::MP_OPEN:
      this->my_tilt_pending_ = false;
      this->set_lift_tilt_(true, false);
      this->start_rx_sync(cover::COVER_OPERATION_OPENING);
      break;
    case iohc_cmd::MP_CLOSE:
      this->my_tilt_pending_ = false;
      this->set_lift_tilt_(false, false);
      this->start_rx_sync(cover::COVER_OPERATION_CLOSING);
      break;
    case iohc_cmd::MP_STOP:
    case iohc_cmd::MP_MY:
      if (this->current_operation == cover::COVER_OPERATION_IDLE &&
          !this->rx_sync_.active() && this->has_my_position_) {
        this->start_rx_sync_to(this->my_position_);
      } else {
        this->stop_rx_sync();
      }
      break;
    default:
      // Unknown / position command: leave UI untouched (discovery already logged).
      break;
  }
}

void SomfyIohcCover::handle_rx_tilt_step_(bool clockwise) {
  this->cancel_1w_tilt_sequence();
  // A physical wheel gesture begins with D200 and therefore stops lift motion.
  // The private direction event arrives afterward and is the point at which we
  // can safely update both the height estimator and the slat angle.
  if (this->rx_sync_.active()) {
    this->stop_rx_sync();
  } else if (this->current_operation != cover::COVER_OPERATION_IDLE) {
    this->recompute_position_();
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->stop_prev_trigger_();
  }
  const int8_t direction = (clockwise != this->tilt_inverted_) ? 1 : -1;
  this->tilt = clamp(this->tilt + direction / static_cast<float>(this->tilt_steps_),
                     0.0f, 1.0f);
  ESP_LOGD(TAG, "RX sync: Venetian tilt %s -> %.0f%%",
           clockwise ? "clockwise" : "counterclockwise", this->tilt * 100.0f);
  this->publish_state();
}

void SomfyIohcCover::start_rx_sync(cover::CoverOperation op) {
  this->my_tilt_pending_ = false;
  this->rx_sync_.start(op == cover::COVER_OPERATION_OPENING, this->position, millis());
  this->current_operation = op;
  this->publish_state();
}

void SomfyIohcCover::start_rx_sync_to(float target_position) {
  this->rx_sync_.start_to(target_position, this->position, millis());
  this->my_tilt_pending_ = this->venetian_;
  if (!this->rx_sync_.active()) {
    this->set_my_tilt_();
    this->my_tilt_pending_ = false;
    return;
  }
  this->current_operation = target_position >= this->position
                                ? cover::COVER_OPERATION_OPENING
                                : cover::COVER_OPERATION_CLOSING;
  this->set_lift_tilt_(
      this->current_operation == cover::COVER_OPERATION_OPENING, false);
  this->publish_state();
}

void SomfyIohcCover::stop_rx_sync() {
  this->rx_sync_.stop();
  this->my_tilt_pending_ = false;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
}

void SomfyIohcCover::loop() {
  if (this->rx_sync_.active()) {
    const uint32_t full_duration_ms = this->rx_sync_.opening() ? this->open_duration_ : this->close_duration_;
    const RxSyncUpdate update = this->rx_sync_.update(millis(), full_duration_ms);

    this->position = update.position;
    if (update.finished) {
      this->rx_sync_.stop();
      this->current_operation = cover::COVER_OPERATION_IDLE;
      if (this->my_tilt_pending_)
        this->set_my_tilt_(false);
      this->my_tilt_pending_ = false;
      this->publish_state();
    } else if (update.publish) {
      this->publish_state();
    }
    return;
  }

  SomfyTimeBasedCover::loop();
}

#endif  // USE_SOMFY_IOHC_RX

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
