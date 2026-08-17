#pragma once

#include <utility>

namespace esphome {

template<typename T> class optional {
 public:
  optional() = default;
  optional(const T &value) : value_(value), has_value_(true) {}  // NOLINT
  bool has_value() const { return this->has_value_; }
  const T &operator*() const { return this->value_; }
  T *operator->() { return &this->value_; }

 protected:
  T value_{};
  bool has_value_{false};
};

template<typename T> T clamp(T value, T min, T max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

}  // namespace esphome
