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
  if (err != ESP_OK) {
    // Never erase NVS automatically. Rolling codes are part of the pairing
    // identity and may be impossible to reconstruct after a blanket erase.
    ESP_LOGE(TAG, "nvs_flash_init failed: %s; refusing to erase pairing state", esp_err_to_name(err));
    return false;
  }

  nvs_initialized = true;
  return true;
}

NVSRollingCodeStorage::NVSRollingCodeStorage(const char *name, const char *key, uint16_t initial_code)
    : name_(name), key_(key), initial_code_(initial_code == 0 ? 1 : initial_code) {}

NVSRollingCodeStorage::~NVSRollingCodeStorage() {
  if (this->opened_)
    nvs_close(this->handle_);
}

bool NVSRollingCodeStorage::open_() {
  if (this->opened_)
    return true;

  if (!ensure_nvs_initialized_())
    return false;

  esp_err_t err = nvs_open(this->name_, NVS_READWRITE, &this->handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open('%s') failed: %s", this->name_, esp_err_to_name(err));
    return false;
  }
  this->opened_ = true;
  return true;
}

bool NVSRollingCodeStorage::read_next_(uint16_t &code) {
  if (!this->open_())
    return false;

  code = this->initial_code_;
  esp_err_t err = nvs_get_u16(this->handle_, this->key_, &code);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    code = this->initial_code_;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_get_u16('%s') failed: %s", this->key_, esp_err_to_name(err));
    return false;
  }
  if (code == 0) {
    ESP_LOGE(TAG, "rolling code '%s' wrapped/exhausted; re-pair with a new identity", this->key_);
    return false;
  }
  return true;
}

uint16_t NVSRollingCodeStorage::peekNextCode() {
  uint16_t code;
  return this->read_next_(code) ? code : 0;
}

bool NVSRollingCodeStorage::seedNextCode(uint16_t code) {
  if (code == 0 || !this->open_())
    return false;
  esp_err_t err = nvs_set_u16(this->handle_, this->key_, code);
  if (err == ESP_OK)
    err = nvs_commit(this->handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "seeding rolling code '%s' failed: %s", this->key_,
             esp_err_to_name(err));
    return false;
  }
  this->initial_code_ = code;
  return true;
}

uint16_t NVSRollingCodeStorage::nextCode() {
  uint16_t code;
  if (!this->read_next_(code))
    return 0;

  esp_err_t err = nvs_set_u16(this->handle_, this->key_, static_cast<uint16_t>(code + 1));
  if (err == ESP_OK)
    err = nvs_commit(this->handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "persisting rolling code '%s' failed: %s", this->key_, esp_err_to_name(err));
    return 0;
  }

  return code;
}
