#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_RTS

#include "somfy_hub_rts.h"
#include "NVSRollingCodeStorage.h"
#include "rx_sync_animator.h"
#include "somfy_time_based_cover.h"
#include "esphome/components/button/button.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include <array>
#include <cinttypes>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>
#include <algorithm>

#ifdef USE_SOMFY_COVER_RX
namespace esphome {
namespace text_sensor {
class TextSensor;
}
}  // namespace esphome
#endif

namespace esphome {
namespace somfy {

// Helper class to attach cover functions to the time based cover triggers
template <typename... Ts> class SomfyCoverAction : public Action<Ts...> {
public:
  std::function<void(Ts...)> callback;
  explicit SomfyCoverAction(std::function<void(Ts...)> callback)
      : callback(callback) {}
  void play(Ts... x) override {
    if (callback)
      callback(x...);
  }
};

class SomfyCover : public SomfyTimeBasedCover {
public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_hub(SomfyRtsHub *hub) { this->hub_ = hub; }

#ifdef USE_SOMFY_COVER_RX
  void add_receive_remote_code(uint32_t code) {
    auto it = std::lower_bound(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), code);
    if (it == this->receive_remote_codes_.end() || *it != code)
      this->receive_remote_codes_.insert(it, code);
  }
  void set_log_text_sensor(text_sensor::TextSensor *ts) { this->log_text_sensor_ = ts; }
#endif
  void set_prog_button(button::Button *btn) { this->cover_prog_button_ = btn; }
  void set_remote_code(uint32_t code) { this->remote_code_ = code; }
  void set_storage_namespace(const char *ns) { this->storage_namespace_ = ns; }
  void set_storage_key(const char *key) { this->storage_key_ = key; }
  void set_initial_rolling_code(uint16_t code) { this->initial_rolling_code_ = code; }
  void set_repeat_count(int count) { this->repeat_count_ = count; }

  cover::CoverTraits get_traits() override;

protected:
  void control(const cover::CoverCall &call) override;

  // Hub reference (owns radio)
  SomfyRtsHub *hub_{nullptr};

  // Per-device config
  button::Button *cover_prog_button_{nullptr};
  uint32_t remote_code_{0};
  const char *storage_namespace_{nullptr};
  const char *storage_key_{nullptr};
  uint16_t initial_rolling_code_{1};
  int repeat_count_{4};

  // Rolling code storage
  std::unique_ptr<NVSRollingCodeStorage> storage_;

  // RX state (for keeping HA in sync with physical remotes)
#ifdef USE_SOMFY_COVER_RX
  std::vector<uint32_t> receive_remote_codes_;
  text_sensor::TextSensor *log_text_sensor_{nullptr};

  // Physical-remote UI animation state.
  RxSyncAnimator rx_sync_;

  // RX handler (registered on hub)
  void on_rts_frame_(const RtsDecodedFrame &frame);
  bool is_allowed_remote_(uint32_t code) const;
  void start_rx_sync_(cover::CoverOperation op);
  void stop_rx_sync_();
#endif

  // TX
  void log_and_send_(const char *label, RtsCommand cmd);
  void build_frame(std::array<uint8_t, 7> &bytes, RtsCommand command, uint16_t code);
  void send_command(RtsCommand command);
  void open();
  void close();
  void stop();
  void program();

  // Automations
  std::unique_ptr<Automation<>> automationTriggerUp_{nullptr};
  std::unique_ptr<SomfyCoverAction<>> actionTriggerUp_{nullptr};
  std::unique_ptr<Automation<>> automationTriggerDown_{nullptr};
  std::unique_ptr<SomfyCoverAction<>> actionTriggerDown_{nullptr};
  std::unique_ptr<Automation<>> automationTriggerStop_{nullptr};
  std::unique_ptr<SomfyCoverAction<>> actionTriggerStop_{nullptr};
};

} // namespace somfy
} // namespace esphome

#endif  // USE_SOMFY_RTS
