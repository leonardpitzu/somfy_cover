#pragma once

#include "esphome/components/button/button.h"
#ifdef USE_SOMFY_COVER_RX
#include "esphome/components/remote_receiver/remote_receiver.h"
#endif
#include "esphome/components/time_based/time_based_cover.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include <vector>
#include <cinttypes>
#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <algorithm>

// Libraries for SomfyRemote
#include "NVSRollingCodeStorage.h"

namespace esphome {
namespace remote_receiver {
class RemoteReceiverComponent;
}
namespace text_sensor {
class TextSensor;
}

namespace somfy_cover {

// Helper class to attach cover functions to the time based cover triggers
template <typename... Ts> class SomfyCoverAction : public Action<Ts...> {
public:
  // The function to be called when the action plays.
  std::function<void(Ts...)> callback;

  explicit SomfyCoverAction(std::function<void(Ts...)> callback)
      : callback(callback) {}

  void play(Ts... x) override {
    if (callback)
      callback(x...);
  }
};

enum class Command : uint8_t {
	My      = 0x1,
	Up      = 0x2,
	MyUp    = 0x3,
	Down    = 0x4,
	MyDown  = 0x5,
	UpDown  = 0x6,
	Prog    = 0x8,
	SunFlag = 0x9,
	Flag    = 0xA
};

struct CoverPosition {
  static constexpr float OPEN = 1.0f;
  static constexpr float CLOSED = 0.0f;
  static constexpr float UNKNOWN = -1.0f;
  static constexpr float MIN_PUBLISH_DELTA = 0.01f;
};

struct SomfyFrame {
  static constexpr size_t SHORT_FRAME_BYTES = 7;
  static constexpr size_t LONG_FRAME_BYTES = 10;
  static constexpr uint8_t SHORT_FRAME_BITS = 56;
  static constexpr uint8_t LONG_FRAME_BITS = 80;

  std::array<uint8_t, LONG_FRAME_BYTES> bytes{};
};

struct SomfyRawFrame {
  uint32_t remote_code{0};
  uint16_t rolling_code{0};
  Command command{Command::My};
};

struct SomfyTiming {
  static constexpr int32_t SYMBOL_USEC = 640;
  static constexpr float TOLERANCE_MIN = 0.7f;
  static constexpr float TOLERANCE_MAX = 1.3f;

  static constexpr uint32_t SOFTWARE_SYNC_USEC = 4850;
  static constexpr int32_t WAKEUP_HIGH_USEC = 9415;
  static constexpr int32_t WAKEUP_LOW_USEC = 9565 + 80000;
  static constexpr int32_t SOFTWARE_SYNC_HIGH_USEC = 4550;
  static constexpr int32_t INTER_FRAME_GAP_USEC = 415 + 30000;

  static constexpr uint8_t FIRST_FRAME_SYNC_COUNT = 2;
  static constexpr uint8_t REPEAT_FRAME_SYNC_COUNT = 7;

  static constexpr uint32_t RX_DEDUP_WINDOW_MS = 150;
  static constexpr uint32_t RX_CACHE_WINDOW_MS = 50;
  static constexpr size_t RX_CACHE_SIGNATURE_LEN = 12;
  static constexpr uint32_t RX_PUBLISH_INTERVAL_MS = 250;
};

class SomfyCover : public time_based::TimeBasedCover
#ifdef USE_SOMFY_COVER_RX
                 , public remote_base::RemoteReceiverListener
#endif
{
public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  #ifdef USE_SOMFY_COVER_RX
    bool on_receive(remote_base::RemoteReceiveData data) override;
  #endif

  void set_remote_transmitter(remote_transmitter::RemoteTransmitterComponent *t) {
    this->remote_transmitter_ = t;
  }

  void set_remote_receiver(remote_receiver::RemoteReceiverComponent *r) {
  #ifdef USE_SOMFY_COVER_RX
    this->remote_receiver_ = r;
  #else
    (void) r;
  #endif
  }
  void add_receive_remote_code(uint32_t code) {
  #ifdef USE_SOMFY_COVER_RX
    // Keep the list sorted/unique so lookup remains O(log N) in RX hot path.
    auto it = std::lower_bound(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), code);
    if (it == this->receive_remote_codes_.end() || *it != code)
      this->receive_remote_codes_.insert(it, code);
  #else
    (void) code;
  #endif
  }
  void set_log_text_sensor(text_sensor::TextSensor *ts) {
  #ifdef USE_SOMFY_COVER_RX
    this->log_text_sensor_ = ts;
  #else
    (void) ts;
  #endif
  }

  // Set somfy cover button and value
  void set_prog_button(button::Button *cover_prog_button) {
    this->cover_prog_button_ = cover_prog_button;
  }
  void set_remote_code(uint32_t remote_code_) {
    this->remote_code_ = remote_code_;
  }
  void set_storage_namespace(const char *storage_namespace_) {
    this->storage_namespace_ = storage_namespace_;
  }
  void set_storage_key(const char *storage_key_) {
    this->storage_key_ = storage_key_;
  }

  void set_repeat_count(int repeat_count_) {
    this->repeat_count_ = repeat_count_;
  }

  cover::CoverTraits get_traits() override;

protected:
  void control(const cover::CoverCall &call) override;

  // Set via the ESPHome yaml
  remote_transmitter::RemoteTransmitterComponent *remote_transmitter_{nullptr};
  button::Button *cover_prog_button_{nullptr};
  uint32_t remote_code_{0};
  const char *storage_namespace_{nullptr};
  const char *storage_key_{nullptr};
  int repeat_count_{4};

  #ifdef USE_SOMFY_COVER_RX
    // Optional receiver path (for keeping HA in sync when physical remotes are used)
    remote_receiver::RemoteReceiverComponent *remote_receiver_{nullptr};
    std::vector<uint32_t> receive_remote_codes_;
    text_sensor::TextSensor *log_text_sensor_{nullptr};
    uint32_t last_rx_ms_{0};

    // RX-derived UI sync (no TX). We simulate time-based movement when a physical remote is used.
    bool rx_sync_active_{false};
    cover::CoverOperation rx_operation_{cover::COVER_OPERATION_IDLE};
    uint32_t rx_start_ms_{0};
    float rx_start_pos_{CoverPosition::CLOSED};
    uint32_t rx_last_publish_ms_{0};
    float rx_last_published_pos_{CoverPosition::UNKNOWN};
  #endif

  // set via the constructor
  std::unique_ptr<NVSRollingCodeStorage> storage_;

  void log_and_send_(const char *label, Command cmd);
  void open();
  void close();
  void stop();
  void program();

  // Create automations to attach the cover control functions
  std::unique_ptr<Automation<>> automationTriggerUp_{nullptr};
  std::unique_ptr<SomfyCoverAction<>> actionTriggerUp{nullptr};
  std::unique_ptr<Automation<>> automationTriggerDown_{nullptr};
  std::unique_ptr<SomfyCoverAction<>> actionTriggerDown_{nullptr};
  std::unique_ptr<Automation<>> automationTriggerStop_{nullptr};
  std::unique_ptr<SomfyCoverAction<>> actionTriggerStop_{nullptr};

  #ifdef USE_SOMFY_COVER_RX
    // rx path
    static const char *command_to_string_(Command cmd);  
    bool is_allowed_remote_(uint32_t code) const;
    bool decode_frame_(const remote_base::RawTimings &data, SomfyRawFrame &decoded_frame, bool debug_log);
  #endif

  // tx path
  void send_high(remote_base::RawTimings & t, int32_t durationInMicroseconds);
  void send_low(remote_base::RawTimings & t, int32_t durationInMicroseconds);
  
  void build_sync(remote_base::RawTimings &t, uint8_t sync);
  void build_data(remote_base::RawTimings &t, const SomfyFrame &frame);
  void build_gap(remote_base::RawTimings &t);
  void build_frame(SomfyFrame &frame, Command command, uint16_t code);

  void send_command(Command command);
};

} // namespace somfy_cover
} // namespace esphome
