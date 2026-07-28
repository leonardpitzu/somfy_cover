#pragma once

#include <cmath>
#include <cstdint>

namespace esphome {
namespace somfy {

/// Result of advancing an RX-sync animation by one loop iteration.
struct RxSyncUpdate {
  /// Extrapolated cover position, clamped to [0.0, 1.0].
  float position;
  /// True when the caller should publish the new position to Home Assistant.
  bool publish;
  /// True when the cover has reached the end stop and the animation is over.
  bool finished;
};

/// Dead-reckoning animator for covers driven by a physical remote.
///
/// When someone presses a button on the remote instead of using Home Assistant,
/// the motor starts moving but reports nothing back (1W protocols have no
/// return channel at all). To keep the entity in sync, the position is
/// extrapolated from the configured travel durations.
///
/// This owns only the arithmetic and the publish throttling. The caller owns
/// the entity and decides when to actually publish, which keeps the logic
/// testable on a host without any of the ESPHome cover machinery.
class RxSyncAnimator {
 public:
  static constexpr float POSITION_OPEN = 1.0f;
  static constexpr float POSITION_CLOSED = 0.0f;
  /// Suppress publishes smaller than this, to avoid flooding the API.
  static constexpr float MIN_PUBLISH_DELTA = 0.01f;
  /// Minimum spacing between position publishes while travelling.
  static constexpr uint32_t PUBLISH_INTERVAL_MS = 250;

  /// Begin animating from @p from_position towards the open or closed end stop.
  void start(bool opening, float from_position, uint32_t now_ms) {
    this->active_ = true;
    this->opening_ = opening;
    this->start_ms_ = now_ms;
    this->start_position_ = from_position;
    this->last_publish_ms_ = 0;
    this->last_published_position_ = -1.0f;
  }

  /// Abort the animation, leaving the cover wherever it currently is.
  void stop() { this->active_ = false; }

  bool active() const { return this->active_; }
  bool opening() const { return this->opening_; }

  /// Advance the animation.
  ///
  /// @param full_duration_ms Time for a full travel in the current direction,
  ///        i.e. open_duration when opening(), close_duration otherwise.
  RxSyncUpdate update(uint32_t now_ms, uint32_t full_duration_ms) {
    const float target = this->opening_ ? POSITION_OPEN : POSITION_CLOSED;

    // Only the remaining travel takes time, so a half-open cover reaches the
    // end stop in half the configured duration.
    float remaining = std::fabs(target - this->start_position_);
    if (remaining > 1.0f)
      remaining = 1.0f;

    const auto duration_ms = static_cast<uint32_t>(static_cast<float>(full_duration_ms) * remaining);
    if (duration_ms == 0) {
      this->active_ = false;
      return RxSyncUpdate{target, false, true};
    }

    const uint32_t elapsed = now_ms - this->start_ms_;
    const float progress =
        (elapsed >= duration_ms) ? 1.0f : (static_cast<float>(elapsed) / static_cast<float>(duration_ms));

    float position = this->start_position_ + (target - this->start_position_) * progress;
    if (position < POSITION_CLOSED)
      position = POSITION_CLOSED;
    if (position > POSITION_OPEN)
      position = POSITION_OPEN;

    if (progress >= 1.0f) {
      this->active_ = false;
      return RxSyncUpdate{position, false, true};
    }

    const bool time_ok = (this->last_publish_ms_ == 0) || ((now_ms - this->last_publish_ms_) >= PUBLISH_INTERVAL_MS);
    const bool delta_ok = (this->last_published_position_ < 0.0f) ||
                          (std::fabs(position - this->last_published_position_) >= MIN_PUBLISH_DELTA);
    const bool publish = time_ok && delta_ok;
    if (publish) {
      this->last_publish_ms_ = now_ms;
      this->last_published_position_ = position;
    }

    return RxSyncUpdate{position, publish, false};
  }

 protected:
  bool active_{false};
  bool opening_{false};
  uint32_t start_ms_{0};
  float start_position_{0.0f};
  uint32_t last_publish_ms_{0};
  float last_published_position_{-1.0f};
};

}  // namespace somfy
}  // namespace esphome
