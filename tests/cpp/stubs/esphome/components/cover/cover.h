#pragma once

#include "esphome/core/helpers.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#define LOG_COVER(prefix, type, obj) ((void) 0)

namespace esphome {
namespace cover {

const float COVER_OPEN = 1.0f;
const float COVER_CLOSED = 0.0f;

enum CoverOperation : uint8_t {
  COVER_OPERATION_IDLE = 0,
  COVER_OPERATION_OPENING,
  COVER_OPERATION_CLOSING,
};

class CoverTraits {
 public:
  void set_supports_stop(bool v) { this->supports_stop_ = v; }
  void set_supports_position(bool v) { this->supports_position_ = v; }
  void set_supports_toggle(bool v) { this->supports_toggle_ = v; }
  void set_supports_tilt(bool v) { this->supports_tilt_ = v; }
  void set_is_assumed_state(bool v) { this->is_assumed_state_ = v; }

  bool get_supports_tilt() const { return this->supports_tilt_; }
  bool get_is_assumed_state() const { return this->is_assumed_state_; }

 protected:
  bool supports_stop_{false};
  bool supports_position_{false};
  bool supports_toggle_{false};
  bool supports_tilt_{false};
  bool is_assumed_state_{false};
};

class CoverCall {
 public:
  CoverCall &set_position(float position) {
    this->position_ = position;
    return *this;
  }
  CoverCall &set_command_stop() {
    this->stop_ = true;
    return *this;
  }
  bool get_stop() const { return this->stop_; }
  const optional<bool> &get_toggle() const { return this->toggle_; }
  const optional<float> &get_position() const { return this->position_; }

 protected:
  optional<float> position_;
  optional<bool> toggle_;
  bool stop_{false};
};

class Cover;

struct CoverRestoreState {
  float position;
  void apply(Cover *cover);
};

class Cover {
 public:
  virtual ~Cover() = default;

  float position{0.0f};
  CoverOperation current_operation{COVER_OPERATION_IDLE};

  virtual CoverTraits get_traits() = 0;

  /// Records every publish so a test can assert on what Home Assistant would see.
  void publish_state(bool save = true) {
    (void) save;
    this->publish_count++;
    this->last_published_position = this->position;
    this->last_published_operation = this->current_operation;
  }

  void get_object_id_to(char *buf) { std::strcpy(buf, "stub_cover"); }

  // Test observability
  int publish_count{0};
  float last_published_position{-1.0f};
  CoverOperation last_published_operation{COVER_OPERATION_IDLE};
  /// When set, setup() restores this position instead of defaulting to 0.5.
  optional<CoverRestoreState> restore_value;

 protected:
  virtual void control(const CoverCall &call) = 0;
  optional<CoverRestoreState> restore_state_() { return this->restore_value; }
};

inline void CoverRestoreState::apply(Cover *cover) {
  cover->position = this->position;
  cover->publish_state();
}

}  // namespace cover
}  // namespace esphome
