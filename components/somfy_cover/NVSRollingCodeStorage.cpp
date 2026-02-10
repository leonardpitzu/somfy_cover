#include "NVSRollingCodeStorage.h"

#include <esp_err.h>
#include <nvs_flash.h>

static bool nvs_initialized = false;

void NVSRollingCodeStorage::ensure_nvs_initialized_() {
  if (nvs_initialized)
    return;

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  nvs_initialized = true;
}

NVSRollingCodeStorage::NVSRollingCodeStorage(const char *name, const char *key)
    : name_(name), key_(key) {}

uint16_t NVSRollingCodeStorage::nextCode() {
  ensure_nvs_initialized_();

  if (!opened_) {
    esp_err_t err = nvs_open(name_, NVS_READWRITE, &handle_);
    ESP_ERROR_CHECK(err);
    opened_ = true;
  }

  uint16_t code = 1;
  esp_err_t err = nvs_get_u16(handle_, key_, &code);

  if (err == ESP_ERR_NVS_NOT_FOUND) {
    code = 1;
  } else if (err != ESP_OK) {
    ESP_ERROR_CHECK(err);
  }

  ESP_ERROR_CHECK(nvs_set_u16(handle_, key_, code + 1));
  ESP_ERROR_CHECK(nvs_commit(handle_));

  return code;
}
