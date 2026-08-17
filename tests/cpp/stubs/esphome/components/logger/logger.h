#pragma once

namespace esphome {
namespace logger {

class Logger {
 public:
  int get_log_level() const { return this->level; }
  int level{3};  // below the DEBUG threshold the hub checks for verbose decode logs
};

extern Logger *global_logger;

}  // namespace logger
}  // namespace esphome
