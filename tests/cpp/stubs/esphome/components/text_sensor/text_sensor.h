#pragma once

#include <string>

namespace esphome {
namespace text_sensor {

class TextSensor {
 public:
  void publish_state(const std::string &state) {
    this->last_state = state;
    this->publish_count++;
  }

  std::string last_state;
  int publish_count{0};
};

}  // namespace text_sensor
}  // namespace esphome
