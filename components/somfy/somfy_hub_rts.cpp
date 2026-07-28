#include "somfy_hub_rts.h"
#include "esphome/core/log.h"
#ifdef USE_SOMFY_COVER_RX
#include "esphome/components/logger/logger.h"
#endif
#include <cmath>

namespace esphome {
namespace somfy {

static const char *TAG = "somfy.rts.hub";

// ---------------------------------------------------------------------------
// Frame struct (internal to TX/RX encoding, not exposed in header)
// ---------------------------------------------------------------------------
struct RtsFrame {
  static constexpr size_t SHORT_FRAME_BYTES = 7;
  static constexpr size_t LONG_FRAME_BYTES = 10;
  static constexpr uint8_t SHORT_FRAME_BITS = 56;
  static constexpr uint8_t LONG_FRAME_BITS = 80;

  std::array<uint8_t, LONG_FRAME_BYTES> bytes{};
};

// ---------------------------------------------------------------------------
// Setup / Loop / Dump
// ---------------------------------------------------------------------------

void SomfyRtsHub::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Somfy RTS hub...");
#ifdef USE_SOMFY_COVER_RX
  if (this->remote_receiver_ != nullptr) {
    ESP_LOGD(TAG, "Registering RX listener on hub");
    this->remote_receiver_->register_listener(this);
  }
#endif
}

void SomfyRtsHub::loop() {}

void SomfyRtsHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Somfy RTS Hub:");
  ESP_LOGCONFIG(TAG, "  Transmitter: %s", this->remote_transmitter_ != nullptr ? "configured" : "MISSING");
#ifdef USE_SOMFY_COVER_RX
  ESP_LOGCONFIG(TAG, "  Receiver: %s", this->remote_receiver_ != nullptr ? "configured" : "not configured");
  ESP_LOGCONFIG(TAG, "  RX callbacks: %u", this->rx_callbacks_.size());
#endif
}

// ---------------------------------------------------------------------------
// TX: Manchester encoding + transmit
// ---------------------------------------------------------------------------

static void push_high(remote_base::RawTimings &t, int32_t us) { t.push_back(us); }
static void push_low(remote_base::RawTimings &t, int32_t us) { t.push_back(-us); }

static void build_sync(remote_base::RawTimings &t, uint8_t sync_count) {
  if (sync_count == RtsTiming::FIRST_FRAME_SYNC_COUNT) {
    push_high(t, RtsTiming::WAKEUP_HIGH_USEC);
    push_low(t, RtsTiming::WAKEUP_LOW_USEC);
  }

  for (int i = 0; i < sync_count; i++) {
    push_high(t, 4 * RtsTiming::SYMBOL_USEC);
    push_low(t, 4 * RtsTiming::SYMBOL_USEC);
  }

  push_high(t, RtsTiming::SOFTWARE_SYNC_HIGH_USEC);
  push_low(t, RtsTiming::SYMBOL_USEC);
}

static void build_data(remote_base::RawTimings &t, const std::array<uint8_t, 7> &bytes) {
  for (uint8_t i = 0; i < RtsFrame::SHORT_FRAME_BITS; i++) {
    if ((bytes[i / 8] >> (7 - (i % 8))) & 1) {
      push_low(t, RtsTiming::SYMBOL_USEC);
      push_high(t, RtsTiming::SYMBOL_USEC);
    } else {
      push_high(t, RtsTiming::SYMBOL_USEC);
      push_low(t, RtsTiming::SYMBOL_USEC);
    }
  }
}

static void build_gap(remote_base::RawTimings &t) {
  push_low(t, RtsTiming::INTER_FRAME_GAP_USEC);
}

void SomfyRtsHub::send_frame(const std::array<uint8_t, 7> &frame_bytes, uint8_t repeat_count) {
  static remote_base::RawTimings tx;
  static remote_base::RawTimings sync;
  static remote_base::RawTimings first_sync;
  static remote_base::RawTimings gap;
  static remote_base::RawTimings data;

  // Build invariant parts once
  if (sync.empty()) {
    build_sync(sync, RtsTiming::REPEAT_FRAME_SYNC_COUNT);
    build_sync(first_sync, RtsTiming::FIRST_FRAME_SYNC_COUNT);
    build_gap(gap);
  }

  // Build data every frame (rolling code changes).
  const size_t expected_data_timings = static_cast<size_t>(RtsFrame::SHORT_FRAME_BITS) * 2;
  if (data.capacity() < expected_data_timings)
    data.reserve(expected_data_timings);
  data.clear();
  build_data(data, frame_bytes);

  const size_t first_frame_size = first_sync.size() + data.size() + gap.size();
  const size_t repeated_frame_size = sync.size() + data.size() + gap.size();
  const size_t required = first_frame_size + repeated_frame_size * static_cast<size_t>(repeat_count);
  if (tx.capacity() < required)
    tx.reserve(required);

  tx.clear();

  tx.insert(tx.end(), first_sync.begin(), first_sync.end());
  tx.insert(tx.end(), data.begin(), data.end());
  tx.insert(tx.end(), gap.begin(), gap.end());

  for (int i = 0; i < repeat_count; i++) {
    tx.insert(tx.end(), sync.begin(), sync.end());
    tx.insert(tx.end(), data.begin(), data.end());
    tx.insert(tx.end(), gap.begin(), gap.end());
  }

  auto call = this->remote_transmitter_->transmit();
  call.get_data()->set_data(tx);
  call.perform();
}

// ---------------------------------------------------------------------------
// RX: Decode + dispatch
// ---------------------------------------------------------------------------

#ifdef USE_SOMFY_COVER_RX

const char *SomfyRtsHub::command_to_string_(RtsCommand cmd) {
  switch (cmd) {
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
}

bool SomfyRtsHub::decode_frame_(const remote_base::RawTimings &data, RtsDecodedFrame &decoded_frame, bool debug_log) {
  const int n = static_cast<int>(data.size());
  if (debug_log) {
    ESP_LOGD(TAG, "decode_frame_ ENTER n=%d first=%d second=%d", n, n > 0 ? data[0] : 0, n > 1 ? data[1] : 0);
  }

  if (n < 20) {
    if (debug_log)
      ESP_LOGD(TAG, "decode_frame_ RETURN FAIL_SHORT n=%d", n);
    return false;
  }

  const uint32_t tempo_synchro_hw_min = static_cast<uint32_t>(RtsTiming::SYMBOL_USEC * 4 * RtsTiming::TOLERANCE_MIN);
  const uint32_t tempo_synchro_hw_max = static_cast<uint32_t>(RtsTiming::SYMBOL_USEC * 4 * RtsTiming::TOLERANCE_MAX);
  const uint32_t tempo_synchro_sw_min = static_cast<uint32_t>(RtsTiming::SOFTWARE_SYNC_USEC * RtsTiming::TOLERANCE_MIN);
  const uint32_t tempo_synchro_sw_max = static_cast<uint32_t>(RtsTiming::SOFTWARE_SYNC_USEC * RtsTiming::TOLERANCE_MAX);
  const uint32_t tempo_half_symbol_min = static_cast<uint32_t>(RtsTiming::SYMBOL_USEC * RtsTiming::TOLERANCE_MIN);
  const uint32_t tempo_half_symbol_max = static_cast<uint32_t>(RtsTiming::SYMBOL_USEC * RtsTiming::TOLERANCE_MAX);
  const uint32_t tempo_symbol_min = static_cast<uint32_t>(RtsTiming::SYMBOL_USEC * 2 * RtsTiming::TOLERANCE_MIN);
  const uint32_t tempo_symbol_max = static_cast<uint32_t>(RtsTiming::SYMBOL_USEC * 2 * RtsTiming::TOLERANCE_MAX);

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

  RtsFrame payload;
  uint8_t cpt_synchro_hw = 0;
  uint8_t cpt_bits = 0;
  uint8_t previous_bit = 0;
  bool waiting_half_symbol = false;
  uint8_t bit_length = RtsFrame::SHORT_FRAME_BITS;

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
    bit_length = RtsFrame::SHORT_FRAME_BITS;
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

          if (cpt_synchro_hw <= 7) bit_length = RtsFrame::SHORT_FRAME_BITS;
          else if (cpt_synchro_hw == 14) bit_length = RtsFrame::SHORT_FRAME_BITS;
          else if (cpt_synchro_hw == 13) bit_length = RtsFrame::LONG_FRAME_BITS;
          else if (cpt_synchro_hw == 12) bit_length = RtsFrame::LONG_FRAME_BITS;
          else if (cpt_synchro_hw > 17) bit_length = RtsFrame::LONG_FRAME_BITS;
          else bit_length = RtsFrame::SHORT_FRAME_BITS;

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
          RtsFrame frame;
          frame.bytes[0] = payload.bytes[0];
          for (size_t k = 1; k < RtsFrame::SHORT_FRAME_BYTES; k++)
            frame.bytes[k] = payload.bytes[k] ^ payload.bytes[k - 1];

          if (bit_length == RtsFrame::LONG_FRAME_BITS) {
            frame.bytes[7] = payload.bytes[7];
            frame.bytes[8] = payload.bytes[8];
            frame.bytes[9] = payload.bytes[9];
          }

          uint8_t checksum = 0;
          for (uint8_t k = 0; k < RtsFrame::SHORT_FRAME_BYTES; k++) {
            if (k == 1) checksum ^= (frame.bytes[k] >> 4);
            else checksum ^= frame.bytes[k] ^ (frame.bytes[k] >> 4);
          }
          checksum &= 0x0F;

          const uint8_t expected = frame.bytes[1] & 0x0F;

          if (checksum == expected) {
            if (bit_length == RtsFrame::LONG_FRAME_BITS) {
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

            decoded_frame.command = static_cast<RtsCommand>(frame.bytes[1] >> 4);
            decoded_frame.rolling_code = (static_cast<uint16_t>(frame.bytes[2]) << 8) | frame.bytes[3];
            decoded_frame.remote_code = (static_cast<uint32_t>(frame.bytes[4]) << 16) |
                                        (static_cast<uint32_t>(frame.bytes[5]) << 8) | frame.bytes[6];

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

bool SomfyRtsHub::on_receive(remote_base::RemoteReceiveData data) {
  const auto &raw = data.get_raw_data();
  const uint32_t now = millis();

  // Basic de-duplication
  if (now - this->last_rx_ms_ < RtsTiming::RX_DEDUP_WINDOW_MS) {
    return false;
  }

  bool dbg = false;
#ifdef USE_LOGGER
  if (logger::global_logger != nullptr) {
    dbg = logger::global_logger->get_log_level() >= 4;
  }
#endif

  if (dbg) {
    ESP_LOGD(TAG, "RX callback: raw_len=%u", (unsigned) raw.size());
  }

  // Cache decode across callbacks
  struct RxCache {
    uint32_t ms{0};
    uint16_t len{0};
    int32_t sig[RtsTiming::RX_CACHE_SIGNATURE_LEN]{0};
    bool valid{false};
    RtsDecodedFrame frame{};
  };
  static RxCache cache;

  auto sig_match = [&](const remote_base::RawTimings &r) -> bool {
    if (!cache.valid) return false;
    if (cache.len != r.size()) return false;
    const size_t m = std::min<size_t>(r.size(), RtsTiming::RX_CACHE_SIGNATURE_LEN);
    for (size_t k = 0; k < m; k++) {
      if (cache.sig[k] != r[k]) return false;
    }
    return true;
  };

  RtsDecodedFrame decoded;
  bool ok = false;

  if (sig_match(raw) && (now - cache.ms) < RtsTiming::RX_CACHE_WINDOW_MS) {
    decoded = cache.frame;
    ok = true;
  } else {
    ok = this->decode_frame_(raw, decoded, dbg);
    if (ok) {
      cache.ms = now;
      cache.len = static_cast<uint16_t>(raw.size());
      const size_t m = std::min<size_t>(raw.size(), RtsTiming::RX_CACHE_SIGNATURE_LEN);
      for (size_t k = 0; k < m; k++) cache.sig[k] = raw[k];
      for (size_t k = m; k < RtsTiming::RX_CACHE_SIGNATURE_LEN; k++) cache.sig[k] = 0;
      cache.frame = decoded;
      cache.valid = true;
    } else {
      cache.valid = false;
    }
  }

  if (!ok)
    return false;

  this->last_rx_ms_ = now;

  if (dbg) {
    ESP_LOGD(TAG, "RX decoded: remote=0x%06" PRIX32 " cmd=%s rolling=0x%04" PRIX16,
             decoded.remote_code, command_to_string_(decoded.command), decoded.rolling_code);
  }

  // Dispatch to all registered callbacks
  for (auto &cb : this->rx_callbacks_) {
    cb(decoded);
  }

  return true;
}

#endif  // USE_SOMFY_COVER_RX

}  // namespace somfy
}  // namespace esphome
