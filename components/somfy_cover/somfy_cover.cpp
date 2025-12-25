#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "somfy_cover.h"
#include <cstdlib>

namespace esphome {
namespace somfy_cover {

static const char *TAG = "somfy_cover.cover";

const char *SomfyCover::command_to_string_(Command cmd) {
  switch (cmd) {
    case Command::My:
      return "MY";
    case Command::Up:
      return "UP";
    case Command::Down:
      return "DOWN";
    case Command::MyUp:
      return "MY_UP";
    case Command::MyDown:
      return "MY_DOWN";
    case Command::UpDown:
      return "UP_DOWN";
    case Command::Prog:
      return "PROG";
    case Command::SunFlag:
      return "SUN_FLAG";
    case Command::Flag:
      return "FLAG";
    default:
      return "UNKNOWN";
  }
}

bool SomfyCover::decode_frame_(const remote_base::RawTimings &data, uint32_t &remote_code, uint16_t &rolling_code, Command &command) {
  // Decoder based on ESPSomfy-RTS (https://github.com/rstrouse/ESPSomfy-RTS) receive state machine (Somfy.cpp/Somfy.h reference):
  // - detect >=4 hardware sync pulses (~4*SYMBOL)
  // - detect software sync (~4850us)
  // - decode 56 bits from pulse widths using the "half-symbol / symbol" accumulator
  // - de-obfuscate (XOR chain) and validate checksum

  const int n = static_cast<int>(data.size());
  // ENTRY LOG (proves decode_frame_ is actually being called for every callback)
  ESP_LOGD(TAG, "decode_frame_ ENTER n=%d first=%d second=%d", n,
           n > 0 ? data[0] : 0, n > 1 ? data[1] : 0);

  if (n < 20) {
    ESP_LOGD(TAG, "decode_frame_ RETURN FAIL_SHORT n=%d", n);
    return false;
  }

  constexpr uint32_t SYMBOL = 640;
  constexpr float TOL_MIN = 0.7f;
  constexpr float TOL_MAX = 1.3f;

  const uint32_t tempo_synchro_hw_min = static_cast<uint32_t>(SYMBOL * 4 * TOL_MIN);
  const uint32_t tempo_synchro_hw_max = static_cast<uint32_t>(SYMBOL * 4 * TOL_MAX);
  const uint32_t tempo_synchro_sw_min = static_cast<uint32_t>(4850 * TOL_MIN);
  const uint32_t tempo_synchro_sw_max = static_cast<uint32_t>(4850 * TOL_MAX);
  const uint32_t tempo_half_symbol_min = static_cast<uint32_t>(SYMBOL * TOL_MIN);
  const uint32_t tempo_half_symbol_max = static_cast<uint32_t>(SYMBOL * TOL_MAX);
  const uint32_t tempo_symbol_min = static_cast<uint32_t>(SYMBOL * 2 * TOL_MIN);
  const uint32_t tempo_symbol_max = static_cast<uint32_t>(SYMBOL * 2 * TOL_MAX);

  auto absu = [](int32_t v) -> uint32_t { return static_cast<uint32_t>(v < 0 ? -v : v); };

  auto calc80Checksum = [](uint8_t b0, uint8_t b1, uint8_t b2) -> uint8_t {
    uint8_t cs80 = 0;
    cs80 = (((b0 & 0xF0) >> 4) ^ ((b1 & 0xF0) >> 4));
    cs80 ^= ((b2 & 0xF0) >> 4);
    cs80 ^= (b0 & 0x0F);
    cs80 ^= (b1 & 0x0F);
    return static_cast<uint8_t>(cs80 & 0x0F);
  };

  enum { WAITING_SYNCHRO = 0, RECEIVING_DATA = 1 } status = WAITING_SYNCHRO;

  uint8_t payload[10]{0};
  uint8_t cpt_synchro_hw = 0;
  uint8_t cpt_bits = 0;
  uint8_t previous_bit = 0;
  bool waiting_half_symbol = false;
  uint8_t bit_length = 56;

  // Debug context
  int last_bad_i = -1;
  uint32_t last_bad_duration = 0;
  bool saw_any_sync = false;
  uint8_t last_sync_hw = 0;
  uint8_t last_sync_bitlen = 0;

  auto reset_to_waiting = [&]() {
    status = WAITING_SYNCHRO;
    cpt_synchro_hw = 0;
    cpt_bits = 0;
    previous_bit = 0;
    waiting_half_symbol = false;
    bit_length = 56;
    memset(payload, 0x00, sizeof(payload));
  };

  reset_to_waiting();

  for (int i = 0; i < n; i++) {
    const uint32_t duration = absu(data[i]);

    switch (status) {
      case WAITING_SYNCHRO: {
        if (duration > tempo_synchro_hw_min && duration < tempo_synchro_hw_max) {
          ++cpt_synchro_hw;
        } else if (duration > tempo_synchro_sw_min && duration < tempo_synchro_sw_max && cpt_synchro_hw >= 4) {
          saw_any_sync = true;
          last_sync_hw = cpt_synchro_hw;

          memset(payload, 0x00, sizeof(payload));
          previous_bit = 0x00;
          waiting_half_symbol = false;
          cpt_bits = 0;

          if (cpt_synchro_hw <= 7) bit_length = 56;
          else if (cpt_synchro_hw == 14) bit_length = 56;
          else if (cpt_synchro_hw == 13) bit_length = 80;
          else if (cpt_synchro_hw == 12) bit_length = 80;
          else if (cpt_synchro_hw > 17) bit_length = 80;
          else bit_length = 56;

          last_sync_bitlen = bit_length;

          ESP_LOGD(TAG, "RX sync ok: hw_sync=%u bit_length=%u start_i=%d n=%d", last_sync_hw, last_sync_bitlen, i + 1, n);

          status = RECEIVING_DATA;
        } else {
          cpt_synchro_hw = 0;
        }
        break;
      }

      case RECEIVING_DATA: {
        if (duration > tempo_symbol_min && duration < tempo_symbol_max && !waiting_half_symbol) {
          previous_bit = 1 - previous_bit;
          payload[cpt_bits / 8] |= static_cast<uint8_t>(previous_bit << (7 - (cpt_bits % 8)));
          ++cpt_bits;
        } else if (duration > tempo_half_symbol_min && duration < tempo_half_symbol_max) {
          if (waiting_half_symbol) {
            waiting_half_symbol = false;
            payload[cpt_bits / 8] |= static_cast<uint8_t>(previous_bit << (7 - (cpt_bits % 8)));
            ++cpt_bits;
          } else {
            waiting_half_symbol = true;
          }
        } else {
          last_bad_i = i;
          last_bad_duration = duration;
          ESP_LOGD(TAG,
                   "RX decode FAIL_RANGE: i=%d d=%u waiting_half=%d bits=%u/%u hw_sync=%u",
                   last_bad_i, last_bad_duration, waiting_half_symbol ? 1 : 0,
                   cpt_bits, bit_length, last_sync_hw);

          reset_to_waiting();
          i -= 1;
          break;
        }

        if (cpt_bits >= bit_length) {
          uint8_t frame[10]{0};
          frame[0] = payload[0];
          for (int k = 1; k < 7; k++)
            frame[k] = payload[k] ^ payload[k - 1];

          if (bit_length == 80) {
            frame[7] = payload[7];
            frame[8] = payload[8];
            frame[9] = payload[9];
          }

          uint8_t checksum = 0;
          for (uint8_t k = 0; k < 7; k++) {
            if (k == 1) checksum ^= (frame[k] >> 4);
            else checksum ^= frame[k] ^ (frame[k] >> 4);
          }
          checksum &= 0x0F;

          const uint8_t expected = frame[1] & 0x0F;

          if (checksum == expected) {
            if (bit_length == 80) {
              const uint8_t want = frame[9] & 0x0F;
              const uint8_t got = calc80Checksum(frame[7], frame[8], frame[9]);
              if (want != got) {
                ESP_LOGD(TAG,
                         "RX decode FAIL_CSUM80: want=%u got=%u hw_sync=%u frame7=0x%02X frame8=0x%02X frame9=0x%02X",
                         want, got, last_sync_hw, frame[7], frame[8], frame[9]);
                reset_to_waiting();
                break;
              }
            }

            command = static_cast<Command>(frame[1] >> 4);
            rolling_code = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
            remote_code = (static_cast<uint32_t>(frame[4]) << 16) |
                          (static_cast<uint32_t>(frame[5]) << 8) |
                          frame[6];

            ESP_LOGD(TAG,
                     "decode_frame_ RETURN OK: remote=0x%06X cmd=0x%X rolling=0x%04X hw_sync=%u bit_length=%u",
                     remote_code, (frame[1] >> 4), rolling_code, last_sync_hw, bit_length);

            return true;
          }

          ESP_LOGD(TAG,
                   "RX decode FAIL_CSUM: cs=%u expected=%u hw_sync=%u bit_length=%u encKey=0x%02X cmdNibble=0x%X",
                   checksum, expected, last_sync_hw, bit_length, frame[0], (frame[1] >> 4));

          reset_to_waiting();
        }

        break;
      }
    }
  }

  if (!saw_any_sync) {
    ESP_LOGD(TAG, "decode_frame_ RETURN FAIL_SYNC (no SW sync found) n=%d", n);
  } else {
    ESP_LOGD(TAG,
             "decode_frame_ RETURN FAIL_END (saw_sync hw_sync=%u bit_length=%u last_bad_i=%d last_bad_d=%u)",
             last_sync_hw, last_sync_bitlen, last_bad_i, last_bad_duration);
  }

  return false;
}

bool SomfyCover::on_receive(remote_base::RemoteReceiveData data) {
  const auto &raw = data.get_raw_data();
  ESP_LOGD(TAG, "RX callback for '%s': raw_len=%u", this->name_.c_str(), (unsigned) raw.size());
  if (!raw.empty()) {
    std::string s;
    const size_t n = std::min<size_t>(raw.size(), 20);
    for (size_t i = 0; i < n; i++) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", (int) raw[i]);
      s += buf;
      if (i + 1 < n) s += ",";
    }
    ESP_LOGD(TAG, "RX first timings: %s", s.c_str());
  }

  // Basic de-duplication: Somfy remotes send repeats, and we may receive multiple frames per press.
  const uint32_t now = millis();
  if (now - this->last_rx_ms_ < 150) {
    return false;
  }

  uint32_t remote_code = 0;
  uint16_t rolling = 0;
  Command cmd = Command::My;

  if (!this->decode_frame_(data.get_raw_data(), remote_code, rolling, cmd))
    return false;

  this->last_rx_ms_ = now;

  const bool is_known_remote = this->receive_remote_codes_.empty() || (std::find(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), remote_code) != this->receive_remote_codes_.end());

  ESP_LOGD(TAG, "RX: remote_code=0x%06" PRIX32 " cmd=%s rolling=0x%04" PRIX16 "%s", remote_code, this->command_to_string_(cmd), rolling, is_known_remote ? " (known)" : "");

  if (this->log_text_sensor_ != nullptr) {
    char buf[96];
    snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04" PRIX16, remote_code, this->command_to_string_(cmd), rolling);
    this->log_text_sensor_->publish_state(buf);
  }

  if (!is_known_remote)
    return true;

// Keep HA UI in sync without transmitting anything.
// We simulate movement using the configured open/close durations so HA doesn't jump instantly.
// NOTE: We *do not* rely on TimeBasedCover's internal movement state here, because this RX update is
// driven externally (physical remote) and we only want UI synchronization.
auto start_rx_move = [&](cover::CoverOperation op) {
  const uint32_t now_ms = millis();
  this->rx_sync_active_ = true;
  this->rx_operation_ = op;
  this->rx_start_ms_ = now_ms;
  this->rx_start_pos_ = this->position;
  this->rx_last_publish_ms_ = 0;
  this->current_operation = op;
  this->publish_state();  // show "opening/closing" immediately (position stays as-is for now)
};

switch (cmd) {
  case Command::Up:
  case Command::MyUp:
    start_rx_move(cover::COVER_OPERATION_OPENING);
    break;

  case Command::Down:
  case Command::MyDown:
    start_rx_move(cover::COVER_OPERATION_CLOSING);
    break;

  case Command::My:
  case Command::UpDown:
    // Stop: keep current position, just stop movement
    this->rx_sync_active_ = false;
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->publish_state();
    break;

  default:
    break;
}

  return true;
}


void SomfyCover::setup() {
  // Setup cover rolling code storage
  this->storage_ = new NVSRollingCodeStorage(this->storage_namespace_, this->storage_key_);

  // Optional receiver support
  if (this->remote_receiver_ != nullptr) {
    ESP_LOGD(TAG, "RX: registering receiver listener for cover '%s'", this->name_.c_str());
    this->remote_receiver_->register_listener(this);
  } else {
    ESP_LOGD(TAG, "RX: no remote_receiver configured for cover '%s'", this->name_.c_str());
  }

  // Attach to timebased cover controls
  automationTriggerUp_ = new Automation<>(this->get_open_trigger());
  actionTriggerUp = new SomfyCoverAction<>([=, this] { return this->open(); });
  automationTriggerUp_->add_action(actionTriggerUp);

  automationTriggerDown_ = new Automation<>(this->get_close_trigger());
  actionTriggerDown_ = new SomfyCoverAction<>([=, this] { return this->close(); });
  automationTriggerDown_->add_action(actionTriggerDown_);

  automationTriggerStop_ = new Automation<>(this->get_stop_trigger());
  actionTriggerStop_ = new SomfyCoverAction<>([=, this] { return this->stop(); });
  automationTriggerStop_->add_action(actionTriggerStop_);

  // Attach the prog button
  this->cover_prog_button_->add_on_press_callback([=, this] { return this->program(); });

  // Set extra settings
  this->has_built_in_endstop_ = true;
  this->assumed_state_ = true;

  TimeBasedCover::setup();
}

void SomfyCover::loop() {
  // If we are syncing from a physical remote press, we simulate motion ourselves.
  // This avoids TimeBasedCover instantly jumping the position (because its internal timers
  // are not started by a CoverCall in this RX-driven path).
  if (this->rx_sync_active_) {
    const uint32_t now_ms = millis();

    // Scale the duration to the *remaining travel distance*.
    // Example: if open_duration is 40s for 0%->100%, and we're at 75% and moving to 100%,
    // the remaining time should be ~10s.
    const uint32_t full_dur_ms = (this->rx_operation_ == cover::COVER_OPERATION_OPENING) ? this->open_duration_ : this->close_duration_;
    float remaining = 1.0f;
    if (this->rx_operation_ == cover::COVER_OPERATION_OPENING) {
      remaining = COVER_OPEN - this->rx_start_pos_;
    } else if (this->rx_operation_ == cover::COVER_OPERATION_CLOSING) {
      remaining = this->rx_start_pos_ - COVER_CLOSED;
    }
    if (remaining < 0.0f)
      remaining = 0.0f;
    if (remaining > 1.0f)
      remaining = 1.0f;

    const uint32_t dur_ms = static_cast<uint32_t>(static_cast<float>(full_dur_ms) * remaining);

    // If duration isn't set (or remaining travel is 0), fall back to immediate end-state.
    if (dur_ms == 0) {
      this->position = (this->rx_operation_ == cover::COVER_OPERATION_OPENING) ? COVER_OPEN : COVER_CLOSED;
      this->rx_sync_active_ = false;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state();
      return;
    }

    const uint32_t elapsed = now_ms - this->rx_start_ms_;
    float progress = (elapsed >= dur_ms) ? 1.0f : (static_cast<float>(elapsed) / static_cast<float>(dur_ms));

    float new_pos = this->rx_start_pos_;
    if (this->rx_operation_ == cover::COVER_OPERATION_OPENING) {
      new_pos = this->rx_start_pos_ + (COVER_OPEN - this->rx_start_pos_) * progress;
    } else if (this->rx_operation_ == cover::COVER_OPERATION_CLOSING) {
      new_pos = this->rx_start_pos_ + (COVER_CLOSED - this->rx_start_pos_) * progress;
    }

    if (new_pos < COVER_CLOSED)
      new_pos = COVER_CLOSED;
    if (new_pos > COVER_OPEN)
      new_pos = COVER_OPEN;

    this->position = new_pos;

    // Publish at a modest rate to keep Wi-Fi/HA traffic low, but smooth enough for UI.
    if (this->rx_last_publish_ms_ == 0 || (now_ms - this->rx_last_publish_ms_) >= 250) {
      this->rx_last_publish_ms_ = now_ms;
      this->publish_state();
    }

    if (progress >= 1.0f) {
      this->rx_sync_active_ = false;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state();
    }
    return;
  }

  // Normal time-based operation (HA-driven control).
  TimeBasedCover::loop();
}

void SomfyCover::dump_config() { ESP_LOGCONFIG(TAG, "Somfy cover"); }

cover::CoverTraits SomfyCover::get_traits() {
  auto traits = TimeBasedCover::get_traits();
  traits.set_supports_tilt(false);

  return traits;
}

void SomfyCover::control(const cover::CoverCall &call) {
  TimeBasedCover::control(call);
}

void SomfyCover::open() {
  std::string command = "OPEN " + this->get_object_id();
  ESP_LOGD("somfy", command.c_str());
  this->send_command(Command::Up);
}

void SomfyCover::close() {
  std::string command = "CLOSE " + this->get_object_id();
  ESP_LOGD("somfy", command.c_str());
  this->send_command(Command::Down);
}

void SomfyCover::stop() {
  std::string command = "STOP " + this->get_object_id();
  ESP_LOGD("somfy", command.c_str());
  this->send_command(Command::My);
}

void SomfyCover::program() {
  std::string command = "PROG " + this->get_object_id();
  ESP_LOGD("somfy", command.c_str());
  this->send_command(Command::Prog);
}

void SomfyCover::send_command(Command command) {
  const uint16_t rollingCode = this->storage_->nextCode();
  uint8_t frame[7];
  build_frame(frame, command, rollingCode);
  remote_base::RawTimings t;
  build_timings(t, frame, 2);
  for (int i = 0; i < this->repeat_count_; i++) {
    build_timings(t, frame, 7);
  }
  auto call = this->remote_transmitter_->transmit();
  call.get_data()->set_data(t);
  call.perform();
}

void SomfyCover::build_frame(uint8_t *frame, Command command, uint16_t code) {
	const uint8_t button = static_cast<uint8_t>(command);
	frame[0] = 0xA7;          // Encryption key. Doesn't matter much
	frame[1] = button << 4;   // Which button did  you press? The 4 LSB will be the checksum
	frame[2] = code >> 8;     // Rolling code (big endian)
	frame[3] = code;          // Rolling code

  frame[4] = this->remote_code_ >> 16;  // Remote address
	frame[5] = this->remote_code_ >> 8;   // Remote address
	frame[6] = this->remote_code_;        // Remote address

	// Checksum calculation: a XOR of all the nibbles
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < 7; i++) {
		checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
	}
	checksum &= 0b1111;  // We keep the last 4 bits only

	// Checksum integration
	frame[1] |= checksum;

	// Obfuscation: a XOR of all the bytes
	for (uint8_t i = 1; i < 7; i++) {
		frame[i] ^= frame[i - 1];
	}
}

void SomfyCover::build_timings(remote_base::RawTimings & t, uint8_t *frame, uint8_t sync) {
  const int32_t SYMBOL = 640;

	if (sync == 2) {  // Only with the first frame.
		// Wake-up pulse & Silence
		send_high(t, 9415);
		send_low(t, 9565 + 80000); // was delay(80)
	}

	// Hardware sync: two sync for the first frame, seven for the following ones.
	for (int i = 0; i < sync; i++) {
		send_high(t, 4 * SYMBOL);
		send_low(t, 4 * SYMBOL);
	}

	// Software sync
	send_high(t, 4550);
	send_low(t, SYMBOL);

	// Data: bits are sent one by one, starting with the MSB.
	for (uint8_t i = 0; i < 56; i++) {
		if (((frame[i / 8] >> (7 - (i % 8))) & 1) == 1) {
			send_low(t, SYMBOL);
			send_high(t, SYMBOL);
		} else {
			send_high(t, SYMBOL);
			send_low(t, SYMBOL);
		}
	}

	// Inter-frame silence
	send_low(t, 415 + 30000); // was delay(30)
}

void SomfyCover::send_high(remote_base::RawTimings & t, int32_t durationUsecs) {
  t.push_back(static_cast<int32_t>(durationUsecs));
}

void SomfyCover::send_low(remote_base::RawTimings & t, int32_t durationUsecs) {
  t.push_back(-static_cast<int32_t>(durationUsecs));
}

} // namespace somfy_cover
} // namespace esphome
