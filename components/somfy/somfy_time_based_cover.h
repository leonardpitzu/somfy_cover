#pragma once

// Shared base for the Somfy cover platforms.
//
// Part 1 is a vendored copy of ESPHome's time_based cover position/trigger
// engine. Since ESPHome 2026.07 esphome::time_based::TimeBasedCover is declared
// `final` and can no longer be used as a base class. The somfy covers only ever
// used it as a convenience for (a) duration-based position estimation and
// (b) the open/close/stop triggers that drive the radio commands. Vendoring that
// logic here keeps the component self-contained and immune to future changes in
// the upstream time_based sub-platform.
//
// Part 2 is the logic both radio platforms (RTS and io-homecontrol) share:
// binding the command triggers to the radio callbacks, and the "a physical
// remote moved the motor" UI animation that keeps Home Assistant in sync.

#include "esphome/core/defines.h"

#include "esphome/components/cover/cover.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <functional>
#include <memory>

// RX state-sync is compiled in as soon as either radio platform enables
// reception, so the shared implementation only exists in builds that use it.
#if defined(USE_SOMFY_COVER_RX) || defined(USE_SOMFY_IOHC_RX)
#define USE_SOMFY_RX_SYNC
#endif

namespace esphome {
namespace somfy {

struct CoverPosition {
  static constexpr float OPEN = 1.0f;
  static constexpr float CLOSED = 0.0f;
  static constexpr float UNKNOWN = -1.0f;
  static constexpr float MIN_PUBLISH_DELTA = 0.01f;
};

/// Action wrapper that runs a plain callback when a cover trigger fires.
class SomfyCommandAction : public Action<> {
 public:
  explicit SomfyCommandAction(std::function<void()> callback) : callback_(std::move(callback)) {}
  void play() override {
    if (this->callback_)
      this->callback_();
  }

 protected:
  std::function<void()> callback_;
};

class SomfyTimeBasedCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  Trigger<> *get_open_trigger() { return &this->open_trigger_; }
  Trigger<> *get_close_trigger() { return &this->close_trigger_; }
  Trigger<> *get_stop_trigger() { return &this->stop_trigger_; }
  void set_open_duration(uint32_t open_duration) { this->open_duration_ = open_duration; }
  void set_close_duration(uint32_t close_duration) { this->close_duration_ = close_duration; }
  cover::CoverTraits get_traits() override;
  void set_has_built_in_endstop(bool value) { this->has_built_in_endstop_ = value; }
  void set_manual_control(bool value) { this->manual_control_ = value; }
  void set_assumed_state(bool value) { this->assumed_state_ = value; }
  cover::CoverOperation get_last_operation() const { return this->last_operation_; }

#ifdef USE_SOMFY_RX_SYNC
  /// Start mirroring a movement that was commanded by a physical remote.
  void start_rx_sync(cover::CoverOperation op);
  /// Stop mirroring and report the cover as idle.
  void stop_rx_sync();
#endif

 protected:
  void control(const cover::CoverCall &call) override;
  void stop_prev_trigger_();
  bool is_at_target_() const;
  void start_direction_(cover::CoverOperation dir);
  void recompute_position_();

  /// Bind the open/close/stop triggers to this platform's radio commands.
  void bind_command_triggers_(std::function<void()> open, std::function<void()> close, std::function<void()> stop);

#ifdef USE_SOMFY_RX_SYNC
  /// Advance the RX animation. Returns true when it handled this loop iteration.
  bool rx_sync_loop_();

  bool rx_sync_active_{false};
  cover::CoverOperation rx_operation_{cover::COVER_OPERATION_IDLE};
  uint32_t rx_start_ms_{0};
  float rx_start_pos_{CoverPosition::CLOSED};
  uint32_t rx_last_publish_ms_{0};
  float rx_last_published_pos_{CoverPosition::UNKNOWN};
#endif

  Trigger<> open_trigger_;
  uint32_t open_duration_{0};
  Trigger<> close_trigger_;
  uint32_t close_duration_{0};
  Trigger<> stop_trigger_;

  std::unique_ptr<Automation<>> open_automation_;
  std::unique_ptr<Automation<>> close_automation_;
  std::unique_ptr<Automation<>> stop_automation_;
  std::unique_ptr<SomfyCommandAction> open_action_;
  std::unique_ptr<SomfyCommandAction> close_action_;
  std::unique_ptr<SomfyCommandAction> stop_action_;

  Trigger<> *prev_command_trigger_{nullptr};
  uint32_t last_recompute_time_{0};
  uint32_t last_publish_time_{0};
  float target_position_{0};
  bool has_built_in_endstop_{false};
  bool manual_control_{false};
  bool assumed_state_{false};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
};

}  // namespace somfy
}  // namespace esphome
