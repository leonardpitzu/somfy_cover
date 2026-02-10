#include "esphome/core/log.h"
#include "esphome/components/logger/logger.h"
#ifdef USE_SOMFY_COVER_RX
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "somfy_cover.h"
#include <cmath>

namespace esphome {
namespace somfy_cover {

static const char *TAG = "somfy_cover.cover";

#ifdef USE_SOMFY_COVER_RX
bool SomfyCover::is_allowed_remote_(uint32_t code) const {
  return this->receive_remote_codes_.empty() ||
         std::binary_search(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), code);
}

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

bool SomfyCover::decode_frame_(const remote_base::RawTimings &data, SomfyRawFrame &decoded_frame, bool debug_log) {
  // Decoder based on ESPSomfy-RTS (https://github.com/rstrouse/ESPSomfy-RTS) receive state machine (Somfy.cpp/Somfy.h reference):
  // - detect >=4 hardware sync pulses (~4*SYMBOL)
  // - detect software sync (~4850us)
  // - decode 56 bits from pulse widths using the "half-symbol / symbol" accumulator
  // - de-obfuscate (XOR chain) and validate checksum

  const int n = static_cast<int>(data.size());
  if (debug_log) {
    // ENTRY LOG (proves decode_frame_ is actually being called for every callback)
    ESP_LOGD(TAG, "decode_frame_ ENTER n=%d first=%d second=%d", n, n > 0 ? data[0] : 0, n > 1 ? data[1] : 0);
  }

  if (n < 20) {
    if (debug_log)
      ESP_LOGD(TAG, "decode_frame_ RETURN FAIL_SHORT n=%d", n);
    return false;
  }

  const uint32_t tempo_synchro_hw_min = static_cast<uint32_t>(SomfyTiming::SYMBOL_USEC * 4 * SomfyTiming::TOLERANCE_MIN);
  const uint32_t tempo_synchro_hw_max = static_cast<uint32_t>(SomfyTiming::SYMBOL_USEC * 4 * SomfyTiming::TOLERANCE_MAX);
  const uint32_t tempo_synchro_sw_min = static_cast<uint32_t>(SomfyTiming::SOFTWARE_SYNC_USEC * SomfyTiming::TOLERANCE_MIN);
  const uint32_t tempo_synchro_sw_max = static_cast<uint32_t>(SomfyTiming::SOFTWARE_SYNC_USEC * SomfyTiming::TOLERANCE_MAX);
  const uint32_t tempo_half_symbol_min = static_cast<uint32_t>(SomfyTiming::SYMBOL_USEC * SomfyTiming::TOLERANCE_MIN);
  const uint32_t tempo_half_symbol_max = static_cast<uint32_t>(SomfyTiming::SYMBOL_USEC * SomfyTiming::TOLERANCE_MAX);
  const uint32_t tempo_symbol_min = static_cast<uint32_t>(SomfyTiming::SYMBOL_USEC * 2 * SomfyTiming::TOLERANCE_MIN);
  const uint32_t tempo_symbol_max = static_cast<uint32_t>(SomfyTiming::SYMBOL_USEC * 2 * SomfyTiming::TOLERANCE_MAX);

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

  SomfyFrame payload;
  uint8_t cpt_synchro_hw = 0;
  uint8_t cpt_bits = 0;
  uint8_t previous_bit = 0;
  bool waiting_half_symbol = false;
  uint8_t bit_length = SomfyFrame::SHORT_FRAME_BITS;

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
    bit_length = SomfyFrame::SHORT_FRAME_BITS;
    payload.bytes.fill(0x00);
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

          payload.bytes.fill(0x00);
          previous_bit = 0x00;
          waiting_half_symbol = false;
          cpt_bits = 0;

          if (cpt_synchro_hw <= 7) bit_length = SomfyFrame::SHORT_FRAME_BITS;
          else if (cpt_synchro_hw == 14) bit_length = SomfyFrame::SHORT_FRAME_BITS;
          else if (cpt_synchro_hw == 13) bit_length = SomfyFrame::LONG_FRAME_BITS;
          else if (cpt_synchro_hw == 12) bit_length = SomfyFrame::LONG_FRAME_BITS;
          else if (cpt_synchro_hw > 17) bit_length = SomfyFrame::LONG_FRAME_BITS;
          else bit_length = SomfyFrame::SHORT_FRAME_BITS;

          last_sync_bitlen = bit_length;

          if (debug_log) {
            ESP_LOGD(TAG, "RX sync ok: hw_sync=%u bit_length=%u start_i=%d n=%d", last_sync_hw, last_sync_bitlen,
                     i + 1, n);
          }

          status = RECEIVING_DATA;
        } else {
          cpt_synchro_hw = 0;
        }
        break;
      }

      case RECEIVING_DATA: {
        if (duration > tempo_symbol_min && duration < tempo_symbol_max && !waiting_half_symbol) {
          previous_bit = 1 - previous_bit;
          payload.bytes[cpt_bits / 8] |= static_cast<uint8_t>(previous_bit << (7 - (cpt_bits % 8)));
          ++cpt_bits;
        } else if (duration > tempo_half_symbol_min && duration < tempo_half_symbol_max) {
          if (waiting_half_symbol) {
            waiting_half_symbol = false;
            payload.bytes[cpt_bits / 8] |= static_cast<uint8_t>(previous_bit << (7 - (cpt_bits % 8)));
            ++cpt_bits;
          } else {
            waiting_half_symbol = true;
          }
        } else {
          last_bad_i = i;
          last_bad_duration = duration;
          if (debug_log) {
            ESP_LOGD(TAG, "RX decode FAIL_RANGE: i=%d d=%u waiting_half=%d bits=%u/%u hw_sync=%u", last_bad_i,
                     last_bad_duration, waiting_half_symbol ? 1 : 0, cpt_bits, bit_length, last_sync_hw);
          }

          reset_to_waiting();
          i -= 1;
          break;
        }

        if (cpt_bits >= bit_length) {
          SomfyFrame frame;
          frame.bytes[0] = payload.bytes[0];
          for (size_t k = 1; k < SomfyFrame::SHORT_FRAME_BYTES; k++)
            frame.bytes[k] = payload.bytes[k] ^ payload.bytes[k - 1];

          if (bit_length == SomfyFrame::LONG_FRAME_BITS) {
            frame.bytes[7] = payload.bytes[7];
            frame.bytes[8] = payload.bytes[8];
            frame.bytes[9] = payload.bytes[9];
          }

          uint8_t checksum = 0;
          for (uint8_t k = 0; k < SomfyFrame::SHORT_FRAME_BYTES; k++) {
            if (k == 1) checksum ^= (frame.bytes[k] >> 4);
            else checksum ^= frame.bytes[k] ^ (frame.bytes[k] >> 4);
          }
          checksum &= 0x0F;

          const uint8_t expected = frame.bytes[1] & 0x0F;

          if (checksum == expected) {
            if (bit_length == SomfyFrame::LONG_FRAME_BITS) {
              const uint8_t want = frame.bytes[9] & 0x0F;
              const uint8_t got = calc80Checksum(frame.bytes[7], frame.bytes[8], frame.bytes[9]);
              if (want != got) {
                if (debug_log) {
                  ESP_LOGD(TAG,
                           "RX decode FAIL_CSUM80: want=%u got=%u hw_sync=%u frame7=0x%02X frame8=0x%02X frame9=0x%02X",
                           want, got, last_sync_hw, frame.bytes[7], frame.bytes[8], frame.bytes[9]);
                }
                reset_to_waiting();
                break;
              }
            }

            decoded_frame.command = static_cast<Command>(frame.bytes[1] >> 4);
            decoded_frame.rolling_code = (static_cast<uint16_t>(frame.bytes[2]) << 8) | frame.bytes[3];
            decoded_frame.remote_code = (static_cast<uint32_t>(frame.bytes[4]) << 16) | (static_cast<uint32_t>(frame.bytes[5]) << 8) | frame.bytes[6];

            if (debug_log) {
              ESP_LOGD(TAG, "decode_frame_ RETURN OK: remote=0x%06X cmd=0x%X rolling=0x%04X hw_sync=%u bit_length=%u",
                       decoded_frame.remote_code, (frame.bytes[1] >> 4), decoded_frame.rolling_code, last_sync_hw,
                       bit_length);
            }

            return true;
          }

          if (debug_log) {
            ESP_LOGD(TAG,
                     "RX decode FAIL_CSUM: cs=%u expected=%u hw_sync=%u bit_length=%u encKey=0x%02X cmdNibble=0x%X",
                     checksum, expected, last_sync_hw, bit_length, frame.bytes[0], (frame.bytes[1] >> 4));
          }

          reset_to_waiting();
        }

        break;
      }
    }
  }

  if (debug_log) {
    if (!saw_any_sync) {
      ESP_LOGD(TAG, "decode_frame_ RETURN FAIL_SYNC (no SW sync found) n=%d", n);
    } else {
      ESP_LOGD(TAG, "decode_frame_ RETURN FAIL_END (saw_sync hw_sync=%u bit_length=%u last_bad_i=%d last_bad_d=%u)",
               last_sync_hw, last_sync_bitlen, last_bad_i, last_bad_duration);
    }
  }

  return false;
}

bool SomfyCover::on_receive(remote_base::RemoteReceiveData data) {
  const auto &raw = data.get_raw_data();
  // Basic de-duplication: Somfy remotes send repeats, and we may receive multiple frames per press.
  const uint32_t now = millis();
  if (now - this->last_rx_ms_ < SomfyTiming::RX_DEDUP_WINDOW_MS) {
    return false;
  }

  // Only do expensive formatting when debug logging is enabled.
  bool dbg = false;
  #ifdef USE_LOGGER
    if (logger::global_logger != nullptr) {
      // DEBUG (and more verbose) enabled when log level is >= 4
      dbg = logger::global_logger->get_log_level() >= 4;
    }
  #endif

  if (dbg) {
    ESP_LOGD(TAG, "RX callback for '%s': raw_len=%u", this->name_.c_str(), (unsigned) raw.size());
    if (!raw.empty()) {
      std::string s;
      const size_t n2 = std::min<size_t>(raw.size(), 20);
      s.reserve(6 * n2);
      for (size_t j = 0; j < n2; j++) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", (int) raw[j]);
        s += buf;
        if (j + 1 < n2) s += ",";
      }
      ESP_LOGD(TAG, "RX first timings: %s", s.c_str());
    }
  }



  // Cache decode across cover instances: the same RF frame is delivered to each cover listener.
  // This avoids decoding the identical raw buffer N times per button press.
  struct RxCache {
    uint32_t ms{0};
    uint16_t len{0};
    int32_t sig[SomfyTiming::RX_CACHE_SIGNATURE_LEN]{0};
    bool valid{false};
    SomfyRawFrame frame{};
  };
  static RxCache cache;

  auto sig_match = [&](const remote_base::RawTimings &r) -> bool {
    if (!cache.valid) return false;
    if (cache.len != r.size()) return false;
    const size_t m = std::min<size_t>(r.size(), SomfyTiming::RX_CACHE_SIGNATURE_LEN);
    for (size_t k = 0; k < m; k++) {
      if (cache.sig[k] != r[k]) return false;
    }
    return true;
  };

  SomfyRawFrame decoded_frame;

  bool decoded = false;
  if (sig_match(raw) && (now - cache.ms) < SomfyTiming::RX_CACHE_WINDOW_MS) {
    decoded_frame = cache.frame;
    decoded = true;
  } else {
    decoded = this->decode_frame_(raw, decoded_frame, dbg);
    if (decoded) {
      cache.ms = now;
      cache.len = static_cast<uint16_t>(raw.size());
      const size_t m = std::min<size_t>(raw.size(), SomfyTiming::RX_CACHE_SIGNATURE_LEN);
      for (size_t k = 0; k < m; k++) cache.sig[k] = raw[k];
      for (size_t k = m; k < SomfyTiming::RX_CACHE_SIGNATURE_LEN; k++) cache.sig[k] = 0;
      cache.frame = decoded_frame;
      cache.valid = true;
    } else {
      cache.valid = false;
   }
  }

  if (!decoded)
    return false;


  this->last_rx_ms_ = now;

  const bool is_known_remote = this->is_allowed_remote_(decoded_frame.remote_code);
  const char *cmd_text = this->command_to_string_(decoded_frame.command);

  if (dbg) {
    ESP_LOGD(TAG, "RX: remote_code=0x%06" PRIX32 " cmd=%s rolling=0x%04" PRIX16 "%s", decoded_frame.remote_code,
             cmd_text, decoded_frame.rolling_code,
             is_known_remote ? " (known)" : "");
  }

  if (this->log_text_sensor_ != nullptr) {
    char buf[96];
    snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04" PRIX16, decoded_frame.remote_code, cmd_text,
             decoded_frame.rolling_code);
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
    this->rx_last_published_pos_ = CoverPosition::UNKNOWN;
    this->current_operation = op;
    this->publish_state();  // show "opening/closing" immediately (position stays as-is for now)
  };

  switch (decoded_frame.command) {
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
#endif


void SomfyCover::setup() {
  // Setup cover rolling code storage
  this->storage_ = std::make_unique<NVSRollingCodeStorage>(this->storage_namespace_, this->storage_key_);

#ifdef USE_SOMFY_COVER_RX
  // Optional receiver support
  if (this->remote_receiver_ != nullptr) {
    ESP_LOGD(TAG, "RX: registering receiver listener for cover '%s'", this->name_.c_str());
    this->remote_receiver_->register_listener(this);
  } else {
    ESP_LOGD(TAG, "RX: no remote_receiver configured for cover '%s'", this->name_.c_str());
  }
#endif

  // Attach to timebased cover controls
  automationTriggerUp_ = std::make_unique<Automation<>>(this->get_open_trigger());
  actionTriggerUp = std::make_unique<SomfyCoverAction<>>([=, this] { return this->open(); });
  automationTriggerUp_->add_action(actionTriggerUp.get());

  automationTriggerDown_ = std::make_unique<Automation<>>(this->get_close_trigger());
  actionTriggerDown_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->close(); });
  automationTriggerDown_->add_action(actionTriggerDown_.get());

  automationTriggerStop_ = std::make_unique<Automation<>>(this->get_stop_trigger());
  actionTriggerStop_ = std::make_unique<SomfyCoverAction<>>([=, this] { return this->stop(); });
  automationTriggerStop_->add_action(actionTriggerStop_.get());

  // Attach the prog button
  this->cover_prog_button_->add_on_press_callback([=, this] { return this->program(); });

  // Set extra settings
  this->has_built_in_endstop_ = true;
  this->assumed_state_ = true;

  TimeBasedCover::setup();
}

void SomfyCover::loop() {
#ifdef USE_SOMFY_COVER_RX
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
      remaining = CoverPosition::OPEN - this->rx_start_pos_;
    } else if (this->rx_operation_ == cover::COVER_OPERATION_CLOSING) {
      remaining = this->rx_start_pos_ - CoverPosition::CLOSED;
    }
    if (remaining < 0.0f)
      remaining = 0.0f;
    if (remaining > 1.0f)
      remaining = 1.0f;

    const uint32_t dur_ms = static_cast<uint32_t>(static_cast<float>(full_dur_ms) * remaining);

    // If duration isn't set (or remaining travel is 0), fall back to immediate end-state.
    if (dur_ms == 0) {
      this->position = (this->rx_operation_ == cover::COVER_OPERATION_OPENING) ? CoverPosition::OPEN : CoverPosition::CLOSED;
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

    if (new_pos < CoverPosition::CLOSED)
      new_pos = CoverPosition::CLOSED;
    if (new_pos > CoverPosition::OPEN)
      new_pos = CoverPosition::OPEN;

    this->position = new_pos;

    // Publish at a modest rate to keep Wi-Fi/HA traffic low, but smooth enough for UI.
    // Also avoid publishing tiny position deltas that don't matter visually.
    const bool time_ok = (this->rx_last_publish_ms_ == 0) || ((now_ms - this->rx_last_publish_ms_) >= SomfyTiming::RX_PUBLISH_INTERVAL_MS);
    const bool delta_ok = (this->rx_last_published_pos_ < CoverPosition::CLOSED) || (std::fabs(this->position - this->rx_last_published_pos_) >= CoverPosition::MIN_PUBLISH_DELTA);
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
#endif

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

void SomfyCover::log_and_send_(const char *label, Command cmd) {
  char object_id[128];
  this->get_object_id_to(object_id);
  ESP_LOGD("somfy", "%s %s", label, object_id);
  this->send_command(cmd);
}

void SomfyCover::open()    { log_and_send_("OPEN", Command::Up);    }
void SomfyCover::close()   { log_and_send_("CLOSE", Command::Down); }
void SomfyCover::stop()    { log_and_send_("STOP", Command::My);    }
void SomfyCover::program() { log_and_send_("PROG", Command::Prog);  }

void SomfyCover::send_high(remote_base::RawTimings & t, int32_t durationUsecs) {
  t.push_back(static_cast<int32_t>(durationUsecs));
}

void SomfyCover::send_low(remote_base::RawTimings & t, int32_t durationUsecs) {
  t.push_back(-static_cast<int32_t>(durationUsecs));
}

void SomfyCover::build_sync(remote_base::RawTimings &t, uint8_t sync) {
  if (sync == SomfyTiming::FIRST_FRAME_SYNC_COUNT) {
    send_high(t, SomfyTiming::WAKEUP_HIGH_USEC);
    send_low(t, SomfyTiming::WAKEUP_LOW_USEC);
  }

  for (int i = 0; i < sync; i++) {
    send_high(t, 4 * SomfyTiming::SYMBOL_USEC);
    send_low(t, 4 * SomfyTiming::SYMBOL_USEC);
  }

  send_high(t, SomfyTiming::SOFTWARE_SYNC_HIGH_USEC);
  send_low(t, SomfyTiming::SYMBOL_USEC);
}

void SomfyCover::build_data(remote_base::RawTimings &t, const SomfyFrame &frame) {
  for (uint8_t i = 0; i < SomfyFrame::SHORT_FRAME_BITS; i++) {
    if ((frame.bytes[i / 8] >> (7 - (i % 8))) & 1) {
      send_low(t, SomfyTiming::SYMBOL_USEC);
      send_high(t, SomfyTiming::SYMBOL_USEC);
    } else {
      send_high(t, SomfyTiming::SYMBOL_USEC);
      send_low(t, SomfyTiming::SYMBOL_USEC);
    }
  }
}

void SomfyCover::build_gap(remote_base::RawTimings &t) {
  send_low(t, SomfyTiming::INTER_FRAME_GAP_USEC);
}

void SomfyCover::build_frame(SomfyFrame &frame, Command command, uint16_t code) {
  frame.bytes.fill(0x00);
  auto &bytes = frame.bytes;

  const uint8_t button = static_cast<uint8_t>(command);
  bytes[0] = 0xA7;          // Encryption key. Doesn't matter much
  bytes[1] = button << 4;   // Which button did  you press? The 4 LSB will be the checksum
  bytes[2] = code >> 8;     // Rolling code (big endian)
  bytes[3] = code;          // Rolling code

  bytes[4] = this->remote_code_ >> 16;  // Remote address
  bytes[5] = this->remote_code_ >> 8;   // Remote address
  bytes[6] = this->remote_code_;        // Remote address

  // Checksum calculation: a XOR of all the nibbles
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < SomfyFrame::SHORT_FRAME_BYTES; i++) {
    checksum = checksum ^ bytes[i] ^ (bytes[i] >> 4);
  }
  checksum &= 0b1111;  // We keep the last 4 bits only

  // Checksum integration
  bytes[1] |= checksum;

  // Obfuscation: a XOR of all the bytes
  for (uint8_t i = 1; i < SomfyFrame::SHORT_FRAME_BYTES; i++) {
    bytes[i] ^= bytes[i - 1];
  }
}

void SomfyCover::send_command(Command command) {
  const uint16_t rollingCode = this->storage_->nextCode();
  SomfyFrame frame;
  build_frame(frame, command, rollingCode);

  static remote_base::RawTimings tx;
  static remote_base::RawTimings sync;
  static remote_base::RawTimings first_sync;
  static remote_base::RawTimings gap;
  static remote_base::RawTimings data;

  // Build invariant parts once
  if (sync.empty()) {
    build_sync(sync, SomfyTiming::REPEAT_FRAME_SYNC_COUNT);
    build_sync(first_sync, SomfyTiming::FIRST_FRAME_SYNC_COUNT);
    build_gap(gap);
  }

  // Build data every frame (rolling code changes).
  const size_t expected_data_timings = static_cast<size_t>(SomfyFrame::SHORT_FRAME_BITS) * 2;
  if (data.capacity() < expected_data_timings)
    data.reserve(expected_data_timings);
  data.clear();
  build_data(data, frame);

  // Reserve enough capacity once to avoid repeated allocations in steady state.
  const size_t first_frame_size = first_sync.size() + data.size() + gap.size();
  const size_t repeated_frame_size = sync.size() + data.size() + gap.size();
  const size_t required_tx_capacity =
      first_frame_size + repeated_frame_size * static_cast<size_t>(this->repeat_count_);
  if (tx.capacity() < required_tx_capacity)
    tx.reserve(required_tx_capacity);

  tx.clear();

  tx.insert(tx.end(), first_sync.begin(), first_sync.end());
  tx.insert(tx.end(), data.begin(), data.end());
  tx.insert(tx.end(), gap.begin(), gap.end());

  // Repeat frames
  for (int i = 0; i < this->repeat_count_; i++) {
    tx.insert(tx.end(), sync.begin(), sync.end());
    tx.insert(tx.end(), data.begin(), data.end());
    tx.insert(tx.end(), gap.begin(), gap.end());
  }

  auto call = this->remote_transmitter_->transmit();
  call.get_data()->set_data(tx);
  call.perform();
}

} // namespace somfy_cover
} // namespace esphome
