#pragma once

#include <cstdint>

#include "esphome/core/hal.h"

namespace esphome {

class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual float get_setup_priority() const { return 0.0f; }
};

}  // namespace esphome
