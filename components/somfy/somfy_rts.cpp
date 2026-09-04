#include "somfy_rts.h"

#ifdef USE_SOMFY_RTS

#include "esphome/core/log.h"
#ifdef USE_SOMFY_COVER_RX
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/hal.h"
#endif
#include <cinttypes>

namespace esphome {
namespace somfy {

static const char *TAG = "somfy.rts";

// ---------------------------------------------------------------------------
// RX callback from hub
// ---------------------------------------------------------------------------

#ifdef USE_SOMFY_COVER_RX

void SomfyCover::on_rts_frame_(const RtsDecodedFrame &frame) {
  // Publish to the discovery text sensor regardless of the allow-list, so an
  // unknown remote can be learned.
  if (this->log_text_sensor_ != nullptr) {
    char buf[96];
    snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04" PRIX16,
             frame.remote_code, rts_command_name(frame.command), frame.rolling_code);
    this->log_text_sensor_->publish_state(buf);
  }

  if (!this->is_allowed_remote_(frame.remote_code))
    return;

  switch (frame.command) {
    case RtsCommand::Up:
    case RtsCommand::MyUp:
      this->start_rx_sync_(cover::COVER_OPERATION_OPENING);
      break;

    case RtsCommand::Down:
    case RtsCommand::MyDown:
      this->start_rx_sync_(cover::COVER_OPERATION_CLOSING);
      break;

    case RtsCommand::My:
    case RtsCommand::UpDown:
      this->stop_rx_sync_();
      break;

    default:
      break;
  }
}

// The motor reports nothing back, so the position is dead-reckoned from the
// configured travel durations for as long as we believe it is moving.
void SomfyCover::start_rx_sync_(cover::CoverOperation op) {
  this->rx_sync_.start(op == cover::COVER_OPERATION_OPENING, this->position, millis());
  this->current_operation = op;
  this->publish_state();
}

void SomfyCover::stop_rx_sync_() {
  this->rx_sync_.stop();
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
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
  this->storage_ = std::make_unique<NVSRollingCodeStorage>(
      this->storage_namespace_, this->storage_key_, this->initial_rolling_code_);

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
  if (this->rx_sync_.active()) {
    const uint32_t full_duration_ms = this->rx_sync_.opening() ? this->open_duration_ : this->close_duration_;
    const RxSyncUpdate update = this->rx_sync_.update(millis(), full_duration_ms);

    this->position = update.position;
    if (update.finished) {
      this->stop_rx_sync_();
    } else if (update.publish) {
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
#ifdef USE_SOMFY_COVER_RX
  // A command from Home Assistant supersedes a physical-remote animation. The
  // animator owns current_operation while it runs, so hand a clean IDLE state to
  // the base machine rather than letting it resume from a stale travel clock.
  if (this->rx_sync_.active()) {
    this->rx_sync_.stop();
    this->current_operation = cover::COVER_OPERATION_IDLE;
  }
#endif

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
  if (rolling_code == 0) {
    ESP_LOGE(TAG, "TX aborted: rolling-code storage unavailable or exhausted");
    return;
  }
  std::array<uint8_t, 7> frame;
  build_frame(frame, command, rolling_code);
  this->hub_->send_frame(frame, static_cast<uint8_t>(this->repeat_count_));
}

} // namespace somfy
} // namespace esphome

#endif  // USE_SOMFY_RTS
