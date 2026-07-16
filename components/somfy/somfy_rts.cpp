#include "somfy_rts.h"
#include "esphome/core/log.h"
#ifdef USE_SOMFY_COVER_RX
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include <cmath>
#include <cinttypes>

namespace esphome {
namespace somfy {

static const char *TAG = "somfy.rts";

// ---------------------------------------------------------------------------
// RX callback from hub
// ---------------------------------------------------------------------------

#ifdef USE_SOMFY_COVER_RX

void SomfyCover::on_rts_frame_(const RtsDecodedFrame &frame) {
  const bool is_known = this->is_allowed_remote_(frame.remote_code);

  // Update text sensor (always, for discovery)
  if (this->log_text_sensor_ != nullptr) {
    static const auto cmd_names = [](RtsCommand c) -> const char * {
      switch (c) {
        case RtsCommand::My:      return "MY";
        case RtsCommand::Up:      return "UP";
        case RtsCommand::Down:    return "DOWN";
        case RtsCommand::MyUp:    return "MY_UP";
        case RtsCommand::MyDown:  return "MY_DOWN";
        case RtsCommand::UpDown:  return "UP_DOWN";
        case RtsCommand::Prog:    return "PROG";
        case RtsCommand::SunFlag: return "SUN_FLAG";
        case RtsCommand::Flag:    return "FLAG";
        default:                  return "UNKNOWN";
      }
    };
    char buf[96];
    snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04" PRIX16,
             frame.remote_code, cmd_names(frame.command), frame.rolling_code);
    this->log_text_sensor_->publish_state(buf);
  }

  if (!is_known)
    return;

  // Keep HA UI in sync — simulate movement using configured durations
  auto start_rx_move = [&](cover::CoverOperation op) {
    const uint32_t now_ms = millis();
    this->rx_sync_active_ = true;
    this->rx_operation_ = op;
    this->rx_start_ms_ = now_ms;
    this->rx_start_pos_ = this->position;
    this->rx_last_publish_ms_ = 0;
    this->rx_last_published_pos_ = CoverPosition::UNKNOWN;
    this->current_operation = op;
    this->publish_state();
  };

  switch (frame.command) {
    case RtsCommand::Up:
    case RtsCommand::MyUp:
      start_rx_move(cover::COVER_OPERATION_OPENING);
      break;

    case RtsCommand::Down:
    case RtsCommand::MyDown:
      start_rx_move(cover::COVER_OPERATION_CLOSING);
      break;

    case RtsCommand::My:
    case RtsCommand::UpDown:
      this->rx_sync_active_ = false;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state();
      break;

    default:
      break;
  }
}

bool SomfyCover::is_allowed_remote_(uint32_t code) const {
  return this->receive_remote_codes_.empty() ||
         std::binary_search(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), code);
}

#endif  // USE_SOMFY_COVER_RX

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void SomfyCover::setup() {
  this->storage_ = std::make_unique<NVSRollingCodeStorage>(this->storage_namespace_, this->storage_key_);

#ifdef USE_SOMFY_COVER_RX
  // Register RX callback on hub (if hub has a receiver)
  this->hub_->register_rx_callback([this](const RtsDecodedFrame &frame) {
    this->on_rts_frame_(frame);
  });
#endif

  // Wire up time-based cover triggers
  automationTriggerUp_ = std::make_unique<Automation<>>(this->get_open_trigger());
  actionTriggerUp_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->open(); });
  automationTriggerUp_->add_action(actionTriggerUp_.get());

  automationTriggerDown_ = std::make_unique<Automation<>>(this->get_close_trigger());
  actionTriggerDown_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->close(); });
  automationTriggerDown_->add_action(actionTriggerDown_.get());

  automationTriggerStop_ = std::make_unique<Automation<>>(this->get_stop_trigger());
  actionTriggerStop_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->stop(); });
  automationTriggerStop_->add_action(actionTriggerStop_.get());

  this->cover_prog_button_->add_on_press_callback([=, this] { return this->program(); });

  this->has_built_in_endstop_ = true;
  this->assumed_state_ = true;

  SomfyTimeBasedCover::setup();
}

// ---------------------------------------------------------------------------
// Loop (RX sync animation)
// ---------------------------------------------------------------------------

void SomfyCover::loop() {
#ifdef USE_SOMFY_COVER_RX
  if (this->rx_sync_active_) {
    const uint32_t now_ms = millis();

    const uint32_t full_dur_ms = (this->rx_operation_ == cover::COVER_OPERATION_OPENING)
                                     ? this->open_duration_
                                     : this->close_duration_;
    float remaining = 1.0f;
    if (this->rx_operation_ == cover::COVER_OPERATION_OPENING) {
      remaining = CoverPosition::OPEN - this->rx_start_pos_;
    } else if (this->rx_operation_ == cover::COVER_OPERATION_CLOSING) {
      remaining = this->rx_start_pos_ - CoverPosition::CLOSED;
    }
    if (remaining < 0.0f) remaining = 0.0f;
    if (remaining > 1.0f) remaining = 1.0f;

    const uint32_t dur_ms = static_cast<uint32_t>(static_cast<float>(full_dur_ms) * remaining);

    if (dur_ms == 0) {
      this->position = (this->rx_operation_ == cover::COVER_OPERATION_OPENING)
                            ? CoverPosition::OPEN
                            : CoverPosition::CLOSED;
      this->rx_sync_active_ = false;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->rx_last_published_pos_ = this->position;
      this->publish_state();
      return;
    }

    const uint32_t elapsed = now_ms - this->rx_start_ms_;
    float progress = (elapsed >= dur_ms) ? 1.0f : (static_cast<float>(elapsed) / static_cast<float>(dur_ms));

    float new_pos = this->rx_start_pos_;
    if (this->rx_operation_ == cover::COVER_OPERATION_OPENING) {
      new_pos = this->rx_start_pos_ + (CoverPosition::OPEN - this->rx_start_pos_) * progress;
    } else if (this->rx_operation_ == cover::COVER_OPERATION_CLOSING) {
      new_pos = this->rx_start_pos_ + (CoverPosition::CLOSED - this->rx_start_pos_) * progress;
    }

    if (new_pos < CoverPosition::CLOSED) new_pos = CoverPosition::CLOSED;
    if (new_pos > CoverPosition::OPEN) new_pos = CoverPosition::OPEN;

    this->position = new_pos;

    const bool time_ok = (this->rx_last_publish_ms_ == 0) ||
                         ((now_ms - this->rx_last_publish_ms_) >= RtsTiming::RX_PUBLISH_INTERVAL_MS);
    const bool delta_ok = (this->rx_last_published_pos_ < CoverPosition::CLOSED) ||
                          (std::fabs(this->position - this->rx_last_published_pos_) >= CoverPosition::MIN_PUBLISH_DELTA);
    if (time_ok && delta_ok) {
      this->rx_last_publish_ms_ = now_ms;
      this->rx_last_published_pos_ = this->position;
      this->publish_state();
    }

    if (progress >= 1.0f) {
      this->rx_sync_active_ = false;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->rx_last_published_pos_ = this->position;
      this->publish_state();
    }
    return;
  }
#endif  // USE_SOMFY_COVER_RX

  SomfyTimeBasedCover::loop();
}

void SomfyCover::dump_config() { ESP_LOGCONFIG(TAG, "Somfy RTS cover"); }

cover::CoverTraits SomfyCover::get_traits() {
  auto traits = SomfyTimeBasedCover::get_traits();
  traits.set_supports_tilt(false);
  return traits;
}

void SomfyCover::control(const cover::CoverCall &call) {
  SomfyTimeBasedCover::control(call);
}

// ---------------------------------------------------------------------------
// TX: frame building + send via hub
// ---------------------------------------------------------------------------

void SomfyCover::log_and_send_(const char *label, RtsCommand cmd) {
  char object_id[128];
  this->get_object_id_to(object_id);
  ESP_LOGD(TAG, "%s %s", label, object_id);
  this->send_command(cmd);
}

void SomfyCover::open()    { log_and_send_("OPEN", RtsCommand::Up);    }
void SomfyCover::close()   { log_and_send_("CLOSE", RtsCommand::Down); }
void SomfyCover::stop()    { log_and_send_("STOP", RtsCommand::My);    }
void SomfyCover::program() { log_and_send_("PROG", RtsCommand::Prog);  }

void SomfyCover::build_frame(std::array<uint8_t, 7> &bytes, RtsCommand command, uint16_t code) {
  bytes.fill(0x00);

  const uint8_t button = static_cast<uint8_t>(command);
  bytes[0] = 0xA7;
  bytes[1] = button << 4;
  bytes[2] = code >> 8;
  bytes[3] = code;
  bytes[4] = this->remote_code_ >> 16;
  bytes[5] = this->remote_code_ >> 8;
  bytes[6] = this->remote_code_;

  // Checksum: XOR of all nibbles
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    checksum = checksum ^ bytes[i] ^ (bytes[i] >> 4);
  }
  checksum &= 0x0F;
  bytes[1] |= checksum;

  // Obfuscation: XOR chain
  for (uint8_t i = 1; i < 7; i++) {
    bytes[i] ^= bytes[i - 1];
  }
}

void SomfyCover::send_command(RtsCommand command) {
  const uint16_t rolling_code = this->storage_->nextCode();
  std::array<uint8_t, 7> frame;
  build_frame(frame, command, rolling_code);
  this->hub_->send_frame(frame, static_cast<uint8_t>(this->repeat_count_));
}

} // namespace somfy
} // namespace esphome
