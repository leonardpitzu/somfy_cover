#pragma once

#include <cstdint>
#include <nvs.h>

#include "RollingCodeStorage.h"

/**
 * Stores the rolling codes in the NVS of an ESP32, the codes require two bytes.
 * NVS is initialized once globally, and the namespace is opened once per instance.
 * Storage failures are logged and fall back to an in-RAM counter so a flash
 * problem never aborts the device mid-command.
 */
class NVSRollingCodeStorage : public RollingCodeStorage {
private:
  const char *name_;
  const char *key_;
  nvs_handle handle_{0};
  bool opened_{false};
  uint16_t last_code_{0};

  static bool ensure_nvs_initialized_();
  uint16_t next_volatile_code_();

public:
  NVSRollingCodeStorage(const char *name, const char *key);
  uint16_t nextCode() override;
};
