#pragma once

#include "esphome/core/hal.h"

namespace esphome {

class Application {
 public:
  uint32_t get_loop_component_start_time() const { return millis(); }
};

extern Application App;

}  // namespace esphome
