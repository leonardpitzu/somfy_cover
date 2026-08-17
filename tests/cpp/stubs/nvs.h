#pragma once

// Minimal stand-in for the ESP-IDF NVS header. Only the handle type is needed
// for NVSRollingCodeStorage.h to parse; the storage implementation itself is
// replaced by an in-memory counter in the test translation unit.

#include <cstdint>

using nvs_handle = uint32_t;
