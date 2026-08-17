#pragma once

#include <cstdint>

namespace esphome {

/// Test-controlled clock. Defined by the test translation unit so a test can
/// advance time deterministically instead of sleeping.
uint32_t millis();

}  // namespace esphome
