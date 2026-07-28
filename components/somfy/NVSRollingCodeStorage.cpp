#include "NVSRollingCodeStorage.h"

#include <esp_err.h>
#include <nvs_flash.h>

#include "esphome/core/log.h"

namespace {

const char *const TAG = "somfy.storage";

bool nvs_initialized = false;

}  // namespace

bool NVSRollingCodeStorage::ensure_nvs_initialized_() {
  if (nvs_initialized)
    return true;

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS needs a reformat (%s), erasing", esp_err_to_name(err));
    err = nvs_flash_erase();
    if (err == ESP_OK)
      err = nvs_flash_init();
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
    return false;
  }

  nvs_initialized = true;
  return true;
}

NVSRollingCodeStorage::NVSRollingCodeStorage(const char *name, const char *key)
    : name_(name), key_(key) {}

uint16_t NVSRollingCodeStorage::next_volatile_code_() {
  // Storage is unavailable. Keep transmitting using an in-RAM counter that only
  // ever moves forward, instead of aborting — ESP_ERROR_CHECK would reboot the
  // device in the middle of a command.
  return ++this->last_code_;
}

uint16_t NVSRollingCodeStorage::nextCode() {
  if (!this->opened_) {
    if (!ensure_nvs_initialized_())
      return this->next_volatile_code_();

    esp_err_t err = nvs_open(this->name_, NVS_READWRITE, &this->handle_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_open('%s') failed: %s", this->name_, esp_err_to_name(err));
      return this->next_volatile_code_();
    }
    this->opened_ = true;
  }

  uint16_t code = 1;
  esp_err_t err = nvs_get_u16(this->handle_, this->key_, &code);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    code = 1;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_get_u16('%s') failed: %s", this->key_, esp_err_to_name(err));
    return this->next_volatile_code_();
  }

  err = nvs_set_u16(this->handle_, this->key_, static_cast<uint16_t>(code + 1));
  if (err == ESP_OK)
    err = nvs_commit(this->handle_);
  if (err != ESP_OK) {
    // The code itself is still valid to send; only persistence failed. That
    // means the next boot may replay codes, so make it loud.
    ESP_LOGE(TAG, "persisting rolling code '%s' failed: %s", this->key_, esp_err_to_name(err));
  }

  this->last_code_ = code;
  return code;
}
