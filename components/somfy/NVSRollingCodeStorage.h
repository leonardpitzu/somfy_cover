#pragma once

#include <cstdint>
#include <nvs.h>

#include "RollingCodeStorage.h"

/**
 * Stores the rolling codes in the NVS of an ESP32, the codes require two bytes.
 * NVS is initialized once globally, and the namespace is opened once per instance.
 * Code 0 is reserved as an error sentinel. A storage failure must abort a
 * transmission: guessing or replaying a rolling code can desynchronise a
 * scarce pairing that is difficult to recover.
 */
class NVSRollingCodeStorage : public RollingCodeStorage {
private:
  const char *name_;
  const char *key_;
  nvs_handle handle_{0};
  bool opened_{false};
  uint16_t initial_code_{1};

  static bool ensure_nvs_initialized_();
  bool open_();
  bool read_next_(uint16_t &code);

public:
  NVSRollingCodeStorage(const char *name, const char *key, uint16_t initial_code = 1);
  ~NVSRollingCodeStorage() override;
  /// Return the next persisted code without consuming it, or 0 on error.
  uint16_t peekNextCode();
  /// Explicitly replace the next persisted code. This is reserved for a
  /// guarded cross-bridge identity transfer into a disabled destination slot;
  /// ordinary commands must only advance through nextCode().
  bool seedNextCode(uint16_t code);
  /// Consume and persist the next code, or return 0 without transmitting on error.
  uint16_t nextCode() override;
};
