#pragma once

#include <functional>
#include <vector>

namespace esphome {

template<typename... Ts> class Action {
 public:
  virtual ~Action() = default;
  virtual void play(Ts... x) = 0;
};

template<typename... Ts> class Trigger {
 public:
  void trigger(Ts... x) {
    for (auto *action : this->actions_)
      action->play(x...);
  }
  void add_action(Action<Ts...> *action) { this->actions_.push_back(action); }
  void stop_action() {}

 protected:
  std::vector<Action<Ts...> *> actions_;
};

template<typename... Ts> class Automation {
 public:
  explicit Automation(Trigger<Ts...> *trigger) : trigger_(trigger) {}
  void add_action(Action<Ts...> *action) { this->trigger_->add_action(action); }

 protected:
  Trigger<Ts...> *trigger_;
};

}  // namespace esphome
