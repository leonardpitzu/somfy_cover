#include "somfy_time_based_cover.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome {
namespace somfy {

using namespace esphome::cover;

static const char *const TAG = "somfy.time_based_cover";

#ifdef USE_SOMFY_RX_SYNC
// Throttle for the RX-driven position animation: at most one state publish per
// interval, and only when the position actually moved.
static constexpr uint32_t RX_SYNC_PUBLISH_INTERVAL_MS = 250;
#endif

void SomfyTimeBasedCover::dump_config() {
  LOG_COVER("", "Somfy Time Based Cover", this);
  ESP_LOGCONFIG(TAG,
                "  Open Duration: %.1fs\n"
                "  Close Duration: %.1fs",
                this->open_duration_ / 1e3f, this->close_duration_ / 1e3f);
}

void SomfyTimeBasedCover::setup() {
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = 0.5f;
  }
}

void SomfyTimeBasedCover::loop() {
#ifdef USE_SOMFY_RX_SYNC
  if (this->rx_sync_loop_())
    return;
#endif

  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  const uint32_t now = App.get_loop_component_start_time();

  // Recompute position every loop cycle
  this->recompute_position_();

  if (this->is_at_target_()) {
    if (this->has_built_in_endstop_ &&
        (this->target_position_ == COVER_OPEN || this->target_position_ == COVER_CLOSED)) {
      // Don't trigger stop, let the cover stop by itself.
      this->current_operation = COVER_OPERATION_IDLE;
    } else {
      this->start_direction_(COVER_OPERATION_IDLE);
    }
    this->publish_state();
  }

  // Send current position every second
  if (now - this->last_publish_time_ > 1000) {
    this->publish_state(false);
    this->last_publish_time_ = now;
  }
}

CoverTraits SomfyTimeBasedCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(true);
  traits.set_supports_tilt(false);
  traits.set_is_assumed_state(this->assumed_state_);
  return traits;
}

void SomfyTimeBasedCover::bind_command_triggers_(std::function<void()> open, std::function<void()> close,
                                                 std::function<void()> stop) {
  this->open_action_ = std::make_unique<SomfyCommandAction>(std::move(open));
  this->open_automation_ = std::make_unique<Automation<>>(&this->open_trigger_);
  this->open_automation_->add_action(this->open_action_.get());

  this->close_action_ = std::make_unique<SomfyCommandAction>(std::move(close));
  this->close_automation_ = std::make_unique<Automation<>>(&this->close_trigger_);
  this->close_automation_->add_action(this->close_action_.get());

  this->stop_action_ = std::make_unique<SomfyCommandAction>(std::move(stop));
  this->stop_automation_ = std::make_unique<Automation<>>(&this->stop_trigger_);
  this->stop_automation_->add_action(this->stop_action_.get());
}

#ifdef USE_SOMFY_RX_SYNC

void SomfyTimeBasedCover::start_rx_sync(CoverOperation op) {
  this->rx_sync_active_ = true;
  this->rx_operation_ = op;
  this->rx_start_ms_ = millis();
  this->rx_start_pos_ = this->position;
  this->rx_last_publish_ms_ = 0;
  this->rx_last_published_pos_ = CoverPosition::UNKNOWN;
  this->current_operation = op;
  this->publish_state();
}

void SomfyTimeBasedCover::stop_rx_sync() {
  this->rx_sync_active_ = false;
  this->current_operation = COVER_OPERATION_IDLE;
  this->publish_state();
}

bool SomfyTimeBasedCover::rx_sync_loop_() {
  if (!this->rx_sync_active_)
    return false;

  const bool opening = this->rx_operation_ == COVER_OPERATION_OPENING;
  const float target = opening ? CoverPosition::OPEN : CoverPosition::CLOSED;
  const uint32_t full_dur_ms = opening ? this->open_duration_ : this->close_duration_;

  // Only the remaining travel takes time, so a cover that is already half open
  // reaches the end stop in half the configured duration.
  const float remaining = clamp(std::fabs(target - this->rx_start_pos_), 0.0f, 1.0f);
  const uint32_t dur_ms = static_cast<uint32_t>(static_cast<float>(full_dur_ms) * remaining);

  if (dur_ms == 0) {
    this->position = target;
    this->rx_last_published_pos_ = this->position;
    this->stop_rx_sync();
    return true;
  }

  const uint32_t now_ms = millis();
  const uint32_t elapsed = now_ms - this->rx_start_ms_;
  const float progress = (elapsed >= dur_ms) ? 1.0f : (static_cast<float>(elapsed) / static_cast<float>(dur_ms));

  this->position = clamp(this->rx_start_pos_ + (target - this->rx_start_pos_) * progress, CoverPosition::CLOSED,
                         CoverPosition::OPEN);

  if (progress >= 1.0f) {
    this->rx_last_published_pos_ = this->position;
    this->stop_rx_sync();
    return true;
  }

  const bool time_ok =
      (this->rx_last_publish_ms_ == 0) || ((now_ms - this->rx_last_publish_ms_) >= RX_SYNC_PUBLISH_INTERVAL_MS);
  const bool delta_ok = (this->rx_last_published_pos_ < CoverPosition::CLOSED) ||
                        (std::fabs(this->position - this->rx_last_published_pos_) >= CoverPosition::MIN_PUBLISH_DELTA);
  if (time_ok && delta_ok) {
    this->rx_last_publish_ms_ = now_ms;
    this->rx_last_published_pos_ = this->position;
    this->publish_state();
  }

  return true;
}

#endif  // USE_SOMFY_RX_SYNC

void SomfyTimeBasedCover::control(const CoverCall &call) {
  if (call.get_stop()) {
    this->start_direction_(COVER_OPERATION_IDLE);
    this->publish_state();
  }
  if (call.get_toggle().has_value()) {
    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->start_direction_(COVER_OPERATION_IDLE);
      this->publish_state();
    } else {
      if (this->position == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING) {
        this->target_position_ = COVER_OPEN;
        this->start_direction_(COVER_OPERATION_OPENING);
      } else {
        this->target_position_ = COVER_CLOSED;
        this->start_direction_(COVER_OPERATION_CLOSING);
      }
    }
  }
  auto pos_val = call.get_position();
  if (pos_val.has_value()) {
    auto pos = *pos_val;
    if (pos == this->position) {
      // already at target
      if (this->manual_control_ && (pos == COVER_OPEN || pos == COVER_CLOSED)) {
        // for covers with manual control switch, we can't rely on the computed position, so if
        // the command triggered again, we'll assume it's in the opposite direction anyway.
        auto op = pos == COVER_CLOSED ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
        this->position = pos == COVER_CLOSED ? COVER_OPEN : COVER_CLOSED;
        this->target_position_ = pos;
        this->start_direction_(op);
      }
      // for covers with built in end stop, we should send the command again
      if (this->has_built_in_endstop_ && (pos == COVER_OPEN || pos == COVER_CLOSED)) {
        auto op = pos == COVER_CLOSED ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
        this->target_position_ = pos;
        this->start_direction_(op);
      }
    } else {
      auto op = pos < this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
      if (this->manual_control_ && (pos == COVER_OPEN || pos == COVER_CLOSED)) {
        this->position = pos == COVER_CLOSED ? COVER_OPEN : COVER_CLOSED;
      }
      this->target_position_ = pos;
      this->start_direction_(op);
    }
  }
}

void SomfyTimeBasedCover::stop_prev_trigger_() {
  if (this->prev_command_trigger_ != nullptr) {
    this->prev_command_trigger_->stop_action();
    this->prev_command_trigger_ = nullptr;
  }
}

bool SomfyTimeBasedCover::is_at_target_() const {
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->position <= this->target_position_;
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void SomfyTimeBasedCover::start_direction_(CoverOperation dir) {
  if (dir == this->current_operation && dir != COVER_OPERATION_IDLE)
    return;

#ifdef USE_SOMFY_RX_SYNC
  // A locally-commanded move takes over from any physical-remote animation,
  // otherwise rx_sync_loop_() would keep overwriting the position.
  this->rx_sync_active_ = false;
#endif

  this->recompute_position_();
  Trigger<> *trig;
  switch (dir) {
    case COVER_OPERATION_IDLE:
      trig = &this->stop_trigger_;
      break;
    case COVER_OPERATION_OPENING:
      this->last_operation_ = dir;
      trig = &this->open_trigger_;
      break;
    case COVER_OPERATION_CLOSING:
      this->last_operation_ = dir;
      trig = &this->close_trigger_;
      break;
    default:
      return;
  }

  this->current_operation = dir;

  this->last_recompute_time_ = millis();

  this->stop_prev_trigger_();
  trig->trigger();
  this->prev_command_trigger_ = trig;
}

void SomfyTimeBasedCover::recompute_position_() {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  float dir;
  float action_dur;
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      break;
    case COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      break;
    default:
      return;
  }

  const uint32_t now = millis();
  this->position += dir * (now - this->last_recompute_time_) / action_dur;
  this->position = clamp(this->position, 0.0f, 1.0f);

  this->last_recompute_time_ = now;
}

}  // namespace somfy
}  // namespace esphome
