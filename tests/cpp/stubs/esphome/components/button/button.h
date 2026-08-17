#pragma once

#include <functional>
#include <utility>
#include <vector>

namespace esphome {
namespace button {

class Button {
 public:
  void add_on_press_callback(std::function<void()> cb) { this->callbacks_.push_back(std::move(cb)); }
  void press() {
    for (auto &cb : this->callbacks_)
      cb();
  }

 protected:
  std::vector<std::function<void()>> callbacks_;
};

}  // namespace button
}  // namespace esphome
