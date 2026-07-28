#include "somfy_rts.h"

#ifdef USE_SOMFY_RTS

#include "esphome/core/log.h"
#ifdef USE_SOMFY_COVER_RX
#include "esphome/components/text_sensor/text_sensor.h"
#include <cinttypes>
#include <cstdio>
#endif

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.rts";

// ---------------------------------------------------------------------------
// RX callback from hub
// ---------------------------------------------------------------------------

#ifdef USE_SOMFY_COVER_RX

void SomfyCover::on_rts_frame_(const RtsDecodedFrame &frame) {
  // Publish every decoded frame, allow-listed or not, so unknown remotes can be
  // discovered from the Home Assistant UI.
  if (this->log_text_sensor_ != nullptr) {
    char buf[96];
    snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04" PRIX16, frame.remote_code,
             rts_command_name(frame.command), frame.rolling_code);
    this->log_text_sensor_->publish_state(buf);
  }

  if (!this->is_allowed_remote_(frame.remote_code))
    return;

  // Keep the HA UI in sync — simulate movement using the configured durations.
  switch (frame.command) {
    case RtsCommand::Up:
    case RtsCommand::MyUp:
      this->start_rx_sync(cover::COVER_OPERATION_OPENING);
      break;

    case RtsCommand::Down:
    case RtsCommand::MyDown:
      this->start_rx_sync(cover::COVER_OPERATION_CLOSING);
      break;

    case RtsCommand::My:
    case RtsCommand::UpDown:
      this->stop_rx_sync();
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
  this->bind_command_triggers_([this] { this->open(); }, [this] { this->close(); }, [this] { this->stop(); });

  this->cover_prog_button_->add_on_press_callback([this] { this->program(); });

  this->has_built_in_endstop_ = true;
  this->assumed_state_ = true;

  SomfyTimeBasedCover::setup();
}

void SomfyCover::dump_config() { ESP_LOGCONFIG(TAG, "Somfy RTS cover"); }

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

#endif  // USE_SOMFY_RTS
