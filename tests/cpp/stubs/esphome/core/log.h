#pragma once

#include <cinttypes>

// No-op logging that still *consumes* its arguments, so variables and TAGs that
// exist only for logging do not produce unused-variable warnings when the real
// component sources are compiled against these stubs.
namespace esphome {
inline void log_sink(const char *tag, ...) { (void) tag; }
}  // namespace esphome

#define ESP_LOGD(tag, ...) ::esphome::log_sink(tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, ...) ::esphome::log_sink(tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, ...) ::esphome::log_sink(tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, ...) ::esphome::log_sink(tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, ...) ::esphome::log_sink(tag, ##__VA_ARGS__)
#define ESP_LOGCONFIG(tag, ...) ::esphome::log_sink(tag, ##__VA_ARGS__)
