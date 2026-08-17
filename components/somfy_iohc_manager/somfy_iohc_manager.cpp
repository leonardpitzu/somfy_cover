#include "somfy_iohc_manager.h"

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include <esp_err.h>
#include <esp_system.h>
#include <mbedtls/gcm.h>
#include <nvs_flash.h>

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.iohc.manager";

namespace {

constexpr uint32_t RECORD_MAGIC = 0x53494F4D;  // "SIOM"
constexpr uint8_t RECORD_VERSION = 1;
constexpr const char *REGISTRY_NAMESPACE = "somfy_mgr";
constexpr uint32_t DISCOVERY_TIMEOUT_MS = 30000;
constexpr uint32_t PAIR_ARM_TIMEOUT_MS = 60000;
constexpr uint8_t BACKUP_FORMAT_VERSION = 1;
constexpr size_t GCM_NONCE_SIZE = 12;
constexpr size_t GCM_TAG_SIZE = 16;
constexpr char GCM_AAD[] = "somfy-io-manager-v1";
constexpr char IMPORTS_BOOTSTRAPPED_KEY[] = "imports_v1";
constexpr char SWAP_JOURNAL_KEY[] = "swap_v1";
constexpr uint16_t FLAG_MOVE_PENDING = 0x8000;
constexpr uint16_t FLAG_MOVE_TARGET_MASK = 0x001F;
constexpr uint32_t SWAP_JOURNAL_MAGIC = 0x53574150;  // "SWAP"
constexpr uint8_t SWAP_JOURNAL_VERSION = 1;

void record_key(uint8_t slot, char output[8]) {
  snprintf(output, 8, "slot%02u", static_cast<unsigned>(slot));
}

std::string hex_encode(const uint8_t *data, size_t length) {
  static constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  std::string output;
  output.resize(length * 2);
  for (size_t i = 0; i < length; i++) {
    output[i * 2] = HEX_DIGITS[data[i] >> 4];
    output[i * 2 + 1] = HEX_DIGITS[data[i] & 0x0F];
  }
  return output;
}

int hex_digit(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  return -1;
}

bool hex_decode(const std::string &input, std::vector<uint8_t> &output) {
  if ((input.size() & 1U) != 0)
    return false;
  output.resize(input.size() / 2);
  for (size_t i = 0; i < output.size(); i++) {
    const int high = hex_digit(input[i * 2]);
    const int low = hex_digit(input[i * 2 + 1]);
    if (high < 0 || low < 0)
      return false;
    output[i] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

}  // namespace

void SomfyIohcManager::set_backup_key(const char *hex_key) {
  this->has_backup_key_ = parse_hex_key_(hex_key, this->backup_key_);
  if (!this->has_backup_key_)
    ESP_LOGE(TAG, "Backup key must be exactly 32 hexadecimal characters");
}

void SomfyIohcManager::add_import(
    uint8_t slot, uint32_t node_id, const char *encryption_key,
    const char *storage_namespace, const char *storage_key,
    uint16_t initial_rolling_code, uint32_t physical_remote,
    uint32_t open_duration_ms, uint32_t close_duration_ms,
    float my_position, uint8_t repeat_count) {
  auto record = default_record_(slot);
  record.state = static_cast<uint8_t>(ManagedSlotState::ACTIVE);
  record.node_id = node_id & 0x00FFFFFF;
  if (!parse_hex_key_(encryption_key, record.encryption_key)) {
    ESP_LOGE(TAG, "Ignoring slot %u import with invalid controller key", slot);
    return;
  }
  record.physical_remote = physical_remote & 0x00FFFFFF;
  record.open_duration_ms = open_duration_ms;
  record.close_duration_ms = close_duration_ms;
  record.my_position_basis_points = static_cast<uint16_t>(
      clamp(my_position, 0.0f, 1.0f) * 10000.0f + 0.5f);
  record.initial_rolling_code = initial_rolling_code == 0 ? 1 : initial_rolling_code;
  record.repeat_count = repeat_count;
  strlcpy(record.storage_namespace, storage_namespace, sizeof(record.storage_namespace));
  strlcpy(record.storage_key, storage_key, sizeof(record.storage_key));
  record.checksum = record_checksum_(record);
  this->imports_.emplace_back(slot, record);
}

void SomfyIohcManager::create_slots() {
  if (!this->slots_.empty())
    return;
  this->max_shutters_ = clamp<uint8_t>(this->max_shutters_, 1, IOHC_MANAGER_MAX_SHUTTERS);
  this->slots_.reserve(this->max_shutters_);
  for (uint8_t index = 0; index < this->max_shutters_; index++) {
    this->slots_.emplace_back();
    auto &slot = this->slots_.back();
    slot.record = default_record_(index);
    snprintf(slot.entity_name, sizeof(slot.entity_name), "Somfy Shutter Slot %02u",
             static_cast<unsigned>(index + 1));
    slot.cover = std::make_unique<SomfyIohcCover>();
    slot.cover->set_hub(this->hub_);
    slot.cover->set_mode(IohcMode::MODE_1W);
    slot.cover->set_runtime_enabled(false);
    slot.cover->set_open_duration(30000);
    slot.cover->set_close_duration(30000);
    slot.cover->set_my_position(0.5f);
    slot.cover->reconfigure_storage(
        slot.record.storage_namespace, slot.record.storage_key,
        slot.record.initial_rolling_code);
    slot.cover->set_rolling_code_callback(
        [this, index](uint16_t next_code) { this->on_rolling_code_(index, next_code); });
    slot.cover->set_remote_command_callback([this, index](uint16_t main_param) {
      char detail[7];
      snprintf(detail, sizeof(detail), "0x%04X", main_param);
      this->publish_status_("remote_command", index, detail);
    });

    // Slot entities are intentionally disabled until the HA commissioning flow
    // confirms a motor jog and enables/renames the chosen one.
    const uint32_t fields =
        (1U << ENTITY_FIELD_DISABLED_BY_DEFAULT_SHIFT) |
        (static_cast<uint32_t>(this->cover_device_class_index_) << ENTITY_FIELD_DC_SHIFT);
    App.register_cover(slot.cover.get(), slot.entity_name, fnv1_hash(slot.entity_name), fields);
  }
}

void SomfyIohcManager::setup() {
  if (this->hub_ == nullptr || this->status_sensor_ == nullptr ||
      this->backup_sensor_ == nullptr || !this->has_backup_key_) {
    ESP_LOGE(TAG, "Manager configuration is incomplete; commissioning is disabled");
    this->mark_failed();
    return;
  }
  if (!this->open_registry_()) {
    this->mark_failed();
    return;
  }

  bool imports_bootstrapped = false;
  if (!this->read_imports_bootstrapped_(imports_bootstrapped)) {
    this->mark_failed();
    return;
  }

  // Load the complete registry before considering YAML imports. This makes it
  // possible to change an import's definitive slot in the same OTA that adds
  // move support: an identity already stored elsewhere is detected globally
  // and never duplicated.
  for (uint8_t index = 0; index < this->slots_.size(); index++) {
    ManagedSlotRecord stored{};
    if (this->load_record_(index, stored)) {
      this->slots_[index].record = stored;
      this->slots_[index].has_record = true;
    }
  }

  if (!imports_bootstrapped) {
    for (const auto &imported : this->imports_) {
      const uint8_t index = imported.first;
      if (index >= this->slots_.size() || this->slots_[index].has_record)
        continue;
      if (this->node_id_in_use_(imported.second.node_id)) {
        ESP_LOGI(TAG, "Import for slot %u already exists in another slot; not duplicating it", index);
        continue;
      }
      this->slots_[index].record = imported.second;
      this->slots_[index].has_record = true;
      if (!this->save_record_(index)) {
        ESP_LOGE(TAG, "Could not persist imported slot %u", index);
        this->mark_failed();
        return;
      }
      ESP_LOGI(TAG, "Imported existing controller into slot %u without pairing", index);
    }
    if (!this->mark_imports_bootstrapped_()) {
      this->mark_failed();
      return;
    }
  }

  this->recover_pending_moves_();
  if (!this->recover_pending_swap_()) {
    this->mark_failed();
    return;
  }

  for (uint8_t index = 0; index < this->slots_.size(); index++) {
    this->apply_slot_(index);
    // Managed covers are registered as entities during generated setup(), but
    // the manager owns their component lifecycle so the pool does not require
    // statically generated component pointers.
    this->slots_[index].cover->runtime_setup();
  }

  this->hub_->register_rx_callback(
      [this](const IohcDecodedPacket &packet) { this->on_iohc_packet_(packet); });

  this->register_service(
      &SomfyIohcManager::commission_service, "somfy_commission", {"action", "slot"});
  this->register_service(
      &SomfyIohcManager::calibrate_service, "somfy_calibrate",
      {"slot", "open_seconds", "close_seconds", "my_percent"});
  this->register_service(
      &SomfyIohcManager::control_service, "somfy_control",
      {"slot", "command", "position_percent"});
  this->register_service(
      &SomfyIohcManager::restore_service, "somfy_restore",
      {"encrypted_backup", "slot"});
  this->register_service(
      &SomfyIohcManager::move_service, "somfy_move", {"slot", "target_slot"});
  this->register_service(
      &SomfyIohcManager::swap_service, "somfy_swap", {"slot", "target_slot"});

  this->publish_status_("ready", -1);
  this->set_timeout("initial-backups", 1000, [this]() {
    for (uint8_t index = 0; index < this->slots_.size(); index++) {
      if (this->slots_[index].has_record)
        this->publish_backup_(index);
    }
  });
}

void SomfyIohcManager::loop() {
  for (auto &slot : this->slots_)
    slot.cover->runtime_loop();

  const uint32_t now = millis();
  if (this->discovery_slot_ >= 0 &&
      static_cast<int32_t>(now - this->discovery_deadline_ms_) >= 0) {
    const int8_t slot = this->discovery_slot_;
    this->discovery_slot_ = -1;
    this->publish_status_("error", slot, "remote_discovery_timeout");
  }
  if (this->armed_slot_ >= 0 &&
      static_cast<int32_t>(now - this->arm_deadline_ms_) >= 0) {
    const int8_t slot = this->armed_slot_;
    this->armed_slot_ = -1;
    this->publish_status_("error", slot, "pairing_arm_expired");
  }
}

void SomfyIohcManager::dump_config() {
  uint8_t active = 0;
  uint8_t uncertain = 0;
  for (const auto &slot : this->slots_) {
    const auto state = static_cast<ManagedSlotState>(slot.record.state);
    if (state == ManagedSlotState::ACTIVE) active++;
    if (state == ManagedSlotState::PAIR_SENT) uncertain++;
  }
  ESP_LOGCONFIG(TAG, "Somfy IO commissioning manager:");
  ESP_LOGCONFIG(TAG, "  Slots: %u", static_cast<unsigned>(this->slots_.size()));
  ESP_LOGCONFIG(TAG, "  Active shutters: %u", active);
  ESP_LOGCONFIG(TAG, "  Uncertain pairing attempts: %u", uncertain);
  ESP_LOGCONFIG(TAG, "  Encrypted recovery backup: enabled");
}

void SomfyIohcManager::commission_service(std::string action, int32_t slot) {
  if (action == "stage") {
    this->stage_slot_(slot);
  } else if (!this->valid_slot_(slot)) {
    this->publish_status_("error", slot, "invalid_slot");
  } else if (action == "discover") {
    this->start_discovery_(static_cast<uint8_t>(slot));
  } else if (action == "arm") {
    this->arm_pairing_(static_cast<uint8_t>(slot));
  } else if (action == "pair") {
    this->transmit_pairing_(static_cast<uint8_t>(slot));
  } else if (action == "confirm") {
    this->confirm_pairing_(static_cast<uint8_t>(slot));
  } else if (action == "discard") {
    this->discard_staged_(static_cast<uint8_t>(slot));
  } else if (action == "query") {
    this->publish_status_("slot", slot);
    this->publish_backup_(static_cast<uint8_t>(slot));
  } else if (action == "export") {
    this->publish_backup_(static_cast<uint8_t>(slot));
    this->publish_status_("backup_exported", slot);
  } else {
    this->publish_status_("error", slot, "unknown_action");
  }
}

void SomfyIohcManager::calibrate_service(
    int32_t slot, float open_seconds, float close_seconds, float my_percent) {
  if (!this->valid_slot_(slot)) {
    this->publish_status_("error", slot, "invalid_slot");
    return;
  }
  auto &managed = this->slots_[slot];
  const auto state = static_cast<ManagedSlotState>(managed.record.state);
  if (state != ManagedSlotState::ACTIVE && state != ManagedSlotState::PAIR_SENT) {
    this->publish_status_("error", slot, "slot_not_paired");
    return;
  }
  if (open_seconds < 1.0f || open_seconds > 300.0f ||
      close_seconds < 1.0f || close_seconds > 300.0f ||
      my_percent < 0.0f || my_percent > 100.0f) {
    this->publish_status_("error", slot, "invalid_calibration");
    return;
  }
  managed.record.open_duration_ms = static_cast<uint32_t>(open_seconds * 1000.0f + 0.5f);
  managed.record.close_duration_ms = static_cast<uint32_t>(close_seconds * 1000.0f + 0.5f);
  managed.record.my_position_basis_points = static_cast<uint16_t>(my_percent * 100.0f + 0.5f);
  if (!this->save_record_(static_cast<uint8_t>(slot))) {
    this->publish_status_("error", slot, "storage_write_failed");
    return;
  }
  this->apply_slot_(static_cast<uint8_t>(slot));
  this->publish_backup_(static_cast<uint8_t>(slot));
  this->publish_status_("calibrated", slot);
}

void SomfyIohcManager::control_service(
    int32_t slot, std::string command, float position_percent) {
  if (!this->valid_slot_(slot)) {
    this->publish_status_("error", slot, "invalid_slot");
    return;
  }
  auto &managed = this->slots_[slot];
  const auto state = static_cast<ManagedSlotState>(managed.record.state);
  if (state != ManagedSlotState::ACTIVE && state != ManagedSlotState::PAIR_SENT) {
    this->publish_status_("error", slot, "slot_not_paired");
    return;
  }
  if (command == "open") managed.cover->runtime_open();
  else if (command == "close") managed.cover->runtime_close();
  else if (command == "stop") managed.cover->runtime_stop();
  else if (command == "my") managed.cover->runtime_my();
  else if (command == "position") {
    managed.cover->runtime_set_position(clamp(position_percent, 0.0f, 100.0f) / 100.0f);
  } else {
    this->publish_status_("error", slot, "unknown_command");
    return;
  }
  this->publish_status_("command_sent", slot, command.c_str());
}

void SomfyIohcManager::restore_service(std::string encrypted_backup, int32_t slot) {
  if (!this->valid_slot_(slot)) {
    this->publish_status_("error", slot, "invalid_slot");
    return;
  }
  if (this->slots_[slot].has_record) {
    this->publish_status_("error", slot, "slot_not_empty");
    return;
  }
  ManagedSlotRecord restored{};
  if (!this->decrypt_record_(encrypted_backup, restored) || !record_is_valid_(restored)) {
    this->publish_status_("error", slot, "invalid_backup");
    return;
  }
  if (restored.initial_rolling_code == 0) {
    this->publish_status_("error", slot, "rolling_code_exhausted");
    return;
  }
  if (this->node_id_in_use_(restored.node_id)) {
    this->publish_status_("error", slot, "controller_id_already_present");
    return;
  }
  // Storage names are local implementation details. A restored identity gets
  // this empty slot's deterministic stream name, bootstrapped from the backup's
  // latest known next code.
  const auto local_defaults = default_record_(static_cast<uint8_t>(slot));
  strlcpy(restored.storage_namespace, local_defaults.storage_namespace,
          sizeof(restored.storage_namespace));
  strlcpy(restored.storage_key, local_defaults.storage_key,
          sizeof(restored.storage_key));
  restored.checksum = record_checksum_(restored);
  this->slots_[slot].record = restored;
  this->slots_[slot].has_record = true;
  if (!this->save_record_(static_cast<uint8_t>(slot))) {
    this->slots_[slot].has_record = false;
    this->publish_status_("error", slot, "storage_write_failed");
    return;
  }
  this->apply_slot_(static_cast<uint8_t>(slot));
  this->publish_backup_(static_cast<uint8_t>(slot));
  this->publish_status_("restored", slot, "ensure_source_bridge_is_off");
}

void SomfyIohcManager::move_service(int32_t slot, int32_t target_slot) {
  if (!this->valid_slot_(slot) || !this->valid_slot_(target_slot)) {
    this->publish_status_("error", slot, "invalid_slot");
    return;
  }
  if (slot == target_slot) {
    this->publish_status_("error", slot, "same_slot");
    return;
  }
  if (this->discovery_slot_ >= 0 || this->armed_slot_ >= 0) {
    this->publish_status_("error", slot, "commissioning_busy");
    return;
  }
  ManagedSwapJournal swap_journal{};
  bool swap_pending = false;
  if (!this->read_swap_journal_(swap_journal, swap_pending) || swap_pending) {
    this->publish_status_("error", slot, "slot_transaction_busy");
    return;
  }

  auto &source = this->slots_[slot];
  const auto source_state = static_cast<ManagedSlotState>(source.record.state);
  const bool pending_same_move =
      source.has_record && source_state == ManagedSlotState::ARCHIVED &&
      (source.record.flags & FLAG_MOVE_PENDING) != 0 &&
      (source.record.flags & FLAG_MOVE_TARGET_MASK) == target_slot;
  if (!pending_same_move) {
    if (!source.has_record || source_state != ManagedSlotState::ACTIVE) {
      this->publish_status_("error", slot, "source_slot_not_active");
      return;
    }
    if (this->slots_[target_slot].has_record) {
      this->publish_status_("error", target_slot, "target_slot_not_empty");
      return;
    }

    const auto original = source.record;
    source.record.state = static_cast<uint8_t>(ManagedSlotState::ARCHIVED);
    source.record.flags = FLAG_MOVE_PENDING |
                          (static_cast<uint16_t>(target_slot) & FLAG_MOVE_TARGET_MASK);
    if (!this->save_record_(static_cast<uint8_t>(slot))) {
      source.record = original;
      this->publish_status_("error", slot, "move_state_save_failed");
      return;
    }
    // The source becomes unable to transmit before a destination copy is made.
    this->apply_slot_(static_cast<uint8_t>(slot));
  }

  if (!this->complete_move_(static_cast<uint8_t>(slot),
                            static_cast<uint8_t>(target_slot))) {
    this->publish_status_("error", slot, "move_interrupted_safe_to_retry");
    return;
  }
  char detail[24];
  snprintf(detail, sizeof(detail), "from_slot_%ld", static_cast<long>(slot + 1));
  this->publish_backup_(static_cast<uint8_t>(target_slot));
  this->publish_status_("moved", target_slot, detail);
}

void SomfyIohcManager::swap_service(int32_t slot, int32_t target_slot) {
  if (!this->valid_slot_(slot) || !this->valid_slot_(target_slot)) {
    this->publish_status_("error", slot, "invalid_slot");
    return;
  }
  if (slot == target_slot) {
    this->publish_status_("error", slot, "same_slot");
    return;
  }
  if (this->discovery_slot_ >= 0 || this->armed_slot_ >= 0) {
    this->publish_status_("error", slot, "commissioning_busy");
    return;
  }
  if (std::any_of(this->slots_.begin(), this->slots_.end(), [](const Slot &candidate) {
        return candidate.has_record &&
               (candidate.record.flags & FLAG_MOVE_PENDING) != 0;
      })) {
    this->publish_status_("error", slot, "slot_transaction_busy");
    return;
  }

  ManagedSwapJournal journal{};
  bool pending = false;
  if (!this->read_swap_journal_(journal, pending)) {
    this->publish_status_("error", slot, "swap_journal_invalid");
    return;
  }
  if (pending) {
    const bool same_pair =
        (journal.first_slot == slot && journal.second_slot == target_slot) ||
        (journal.first_slot == target_slot && journal.second_slot == slot);
    if (!same_pair) {
      this->publish_status_("error", slot, "slot_transaction_busy");
      return;
    }
  } else {
    const auto &first = this->slots_[slot];
    const auto &second = this->slots_[target_slot];
    if (!first.has_record || !second.has_record ||
        static_cast<ManagedSlotState>(first.record.state) != ManagedSlotState::ACTIVE ||
        static_cast<ManagedSlotState>(second.record.state) != ManagedSlotState::ACTIVE) {
      this->publish_status_("error", slot, "both_slots_must_be_active");
      return;
    }
    journal.magic = SWAP_JOURNAL_MAGIC;
    journal.version = SWAP_JOURNAL_VERSION;
    journal.first_slot = static_cast<uint8_t>(slot);
    journal.second_slot = static_cast<uint8_t>(target_slot);
    journal.first_record = first.record;
    journal.second_record = second.record;
    if (!this->save_swap_journal_(journal)) {
      this->publish_status_("error", slot, "swap_journal_save_failed");
      return;
    }
  }

  // Once the journal is durable, neither pre-swap runtime object may transmit
  // while the two NVS destination records are being replaced.
  this->slots_[journal.first_slot].cover->set_runtime_enabled(false);
  this->slots_[journal.second_slot].cover->set_runtime_enabled(false);
  if (!this->complete_swap_(journal)) {
    this->publish_status_("error", slot, "swap_interrupted_safe_to_retry");
    return;
  }

  char detail[24];
  snprintf(detail, sizeof(detail), "with_slot_%ld", static_cast<long>(target_slot + 1));
  this->publish_status_("swapped", slot, detail);
}

bool SomfyIohcManager::open_registry_() {
  const esp_err_t init = nvs_flash_init();
  if (init != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %s; refusing to erase controller data",
             esp_err_to_name(init));
    return false;
  }
  const esp_err_t open = nvs_open(REGISTRY_NAMESPACE, NVS_READWRITE, &this->registry_handle_);
  if (open != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(open));
    return false;
  }
  this->registry_open_ = true;
  return true;
}

bool SomfyIohcManager::read_imports_bootstrapped_(bool &bootstrapped) {
  bootstrapped = false;
  if (!this->registry_open_)
    return false;
  uint8_t value = 0;
  const esp_err_t err = nvs_get_u8(this->registry_handle_, IMPORTS_BOOTSTRAPPED_KEY, &value);
  if (err == ESP_ERR_NVS_NOT_FOUND)
    return true;
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Reading import-bootstrap marker failed: %s", esp_err_to_name(err));
    return false;
  }
  bootstrapped = value == 1;
  return true;
}

bool SomfyIohcManager::mark_imports_bootstrapped_() {
  if (!this->registry_open_)
    return false;
  esp_err_t err = nvs_set_u8(this->registry_handle_, IMPORTS_BOOTSTRAPPED_KEY, 1);
  if (err == ESP_OK)
    err = nvs_commit(this->registry_handle_);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "Saving import-bootstrap marker failed: %s", esp_err_to_name(err));
  return err == ESP_OK;
}

bool SomfyIohcManager::load_record_(uint8_t slot, ManagedSlotRecord &record) {
  if (!this->registry_open_)
    return false;
  char key[8];
  record_key(slot, key);
  size_t size = sizeof(record);
  const esp_err_t err = nvs_get_blob(this->registry_handle_, key, &record, &size);
  if (err == ESP_ERR_NVS_NOT_FOUND)
    return false;
  if (err != ESP_OK || size != sizeof(record) || !record_is_valid_(record)) {
    ESP_LOGE(TAG, "Slot %u registry record is invalid; preserving it and disabling the slot", slot);
    // Treat corrupt-but-present storage as occupied. Returning false here would
    // let a YAML import or a later commissioning attempt overwrite controller
    // material that may still be recoverable from the raw NVS blob.
    record = default_record_(slot);
    record.state = static_cast<uint8_t>(ManagedSlotState::ARCHIVED);
    record.checksum = record_checksum_(record);
    return true;
  }
  return true;
}

bool SomfyIohcManager::save_record_(uint8_t slot) {
  if (!this->registry_open_ || !this->valid_slot_(slot))
    return false;
  auto &record = this->slots_[slot].record;
  record.magic = RECORD_MAGIC;
  record.version = RECORD_VERSION;
  record.checksum = record_checksum_(record);
  char key[8];
  record_key(slot, key);
  esp_err_t err = nvs_set_blob(this->registry_handle_, key, &record, sizeof(record));
  if (err == ESP_OK)
    err = nvs_commit(this->registry_handle_);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "Saving slot %u failed: %s", slot, esp_err_to_name(err));
  return err == ESP_OK;
}

bool SomfyIohcManager::erase_record_(uint8_t slot) {
  if (!this->registry_open_)
    return false;
  char key[8];
  record_key(slot, key);
  esp_err_t err = nvs_erase_key(this->registry_handle_, key);
  if (err == ESP_ERR_NVS_NOT_FOUND)
    return true;
  if (err == ESP_OK)
    err = nvs_commit(this->registry_handle_);
  return err == ESP_OK;
}

bool SomfyIohcManager::read_swap_journal_(ManagedSwapJournal &journal, bool &found) {
  found = false;
  if (!this->registry_open_)
    return false;
  size_t size = sizeof(journal);
  const esp_err_t err =
      nvs_get_blob(this->registry_handle_, SWAP_JOURNAL_KEY, &journal, &size);
  if (err == ESP_ERR_NVS_NOT_FOUND)
    return true;
  if (err != ESP_OK || size != sizeof(journal) || !swap_is_valid_(journal)) {
    ESP_LOGE(TAG, "Swap journal is invalid; preserving it and disabling commissioning");
    return false;
  }
  found = true;
  return true;
}

bool SomfyIohcManager::save_swap_journal_(ManagedSwapJournal &journal) {
  if (!this->registry_open_)
    return false;
  journal.checksum = swap_checksum_(journal);
  esp_err_t err = nvs_set_blob(this->registry_handle_, SWAP_JOURNAL_KEY, &journal,
                               sizeof(journal));
  if (err == ESP_OK)
    err = nvs_commit(this->registry_handle_);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "Saving swap journal failed: %s", esp_err_to_name(err));
  return err == ESP_OK;
}

bool SomfyIohcManager::erase_swap_journal_() {
  if (!this->registry_open_)
    return false;
  esp_err_t err = nvs_erase_key(this->registry_handle_, SWAP_JOURNAL_KEY);
  if (err == ESP_ERR_NVS_NOT_FOUND)
    return true;
  if (err == ESP_OK)
    err = nvs_commit(this->registry_handle_);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "Erasing swap journal failed: %s", esp_err_to_name(err));
  return err == ESP_OK;
}

ManagedSlotRecord SomfyIohcManager::default_record_(uint8_t slot) {
  ManagedSlotRecord record{};
  record.magic = RECORD_MAGIC;
  record.version = RECORD_VERSION;
  record.state = static_cast<uint8_t>(ManagedSlotState::EMPTY);
  record.open_duration_ms = 30000;
  record.close_duration_ms = 30000;
  record.my_position_basis_points = 5000;
  record.initial_rolling_code = 1;
  record.repeat_count = 6;
  strlcpy(record.storage_namespace, REGISTRY_NAMESPACE, sizeof(record.storage_namespace));
  snprintf(record.storage_key, sizeof(record.storage_key), "r%02u", static_cast<unsigned>(slot));
  record.checksum = record_checksum_(record);
  return record;
}

uint32_t SomfyIohcManager::record_checksum_(const ManagedSlotRecord &record) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&record);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < offsetof(ManagedSlotRecord, checksum); i++) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

uint32_t SomfyIohcManager::swap_checksum_(const ManagedSwapJournal &journal) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&journal);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < offsetof(ManagedSwapJournal, checksum); i++) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool SomfyIohcManager::record_is_valid_(const ManagedSlotRecord &record) {
  const bool flags_valid =
      record.flags == 0 ||
      ((record.flags & FLAG_MOVE_PENDING) != 0 &&
       (record.flags & ~(FLAG_MOVE_PENDING | FLAG_MOVE_TARGET_MASK)) == 0 &&
       (record.flags & FLAG_MOVE_TARGET_MASK) < IOHC_MANAGER_MAX_SHUTTERS);
  if (record.magic != RECORD_MAGIC || record.version != RECORD_VERSION ||
      record.state > static_cast<uint8_t>(ManagedSlotState::ARCHIVED) ||
      record.node_id > 0x00FFFFFF || record.storage_namespace[15] != '\0' ||
      record.storage_key[15] != '\0' || !flags_valid) {
    return false;
  }
  return record.checksum == record_checksum_(record);
}

bool SomfyIohcManager::swap_is_valid_(const ManagedSwapJournal &journal) {
  return journal.magic == SWAP_JOURNAL_MAGIC &&
         journal.version == SWAP_JOURNAL_VERSION &&
         journal.first_slot < IOHC_MANAGER_MAX_SHUTTERS &&
         journal.second_slot < IOHC_MANAGER_MAX_SHUTTERS &&
         journal.first_slot != journal.second_slot &&
         record_is_valid_(journal.first_record) &&
         record_is_valid_(journal.second_record) &&
         static_cast<ManagedSlotState>(journal.first_record.state) ==
             ManagedSlotState::ACTIVE &&
         static_cast<ManagedSlotState>(journal.second_record.state) ==
             ManagedSlotState::ACTIVE &&
         journal.first_record.flags == 0 && journal.second_record.flags == 0 &&
         journal.checksum == swap_checksum_(journal);
}

const char *SomfyIohcManager::state_name_(ManagedSlotState state) {
  switch (state) {
    case ManagedSlotState::EMPTY: return "empty";
    case ManagedSlotState::STAGED: return "staged";
    case ManagedSlotState::PAIR_SENT: return "pair_sent";
    case ManagedSlotState::ACTIVE: return "active";
    case ManagedSlotState::ARCHIVED: return "archived";
    default: return "invalid";
  }
}

bool SomfyIohcManager::parse_hex_key_(const char *value, uint8_t key[16]) {
  if (value == nullptr || strlen(value) != 32)
    return false;
  for (size_t index = 0; index < 16; index++) {
    const int high = hex_digit(value[index * 2]);
    const int low = hex_digit(value[index * 2 + 1]);
    if (high < 0 || low < 0)
      return false;
    key[index] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

bool SomfyIohcManager::valid_slot_(int32_t slot) const {
  return slot >= 0 && slot < static_cast<int32_t>(this->slots_.size());
}

int8_t SomfyIohcManager::find_empty_slot_() const {
  for (uint8_t index = 0; index < this->slots_.size(); index++) {
    if (!this->slots_[index].has_record)
      return static_cast<int8_t>(index);
  }
  return -1;
}

bool SomfyIohcManager::node_id_in_use_(uint32_t node_id) const {
  return std::any_of(this->slots_.begin(), this->slots_.end(), [node_id](const Slot &slot) {
    return slot.has_record && slot.record.node_id == (node_id & 0x00FFFFFF);
  });
}

void SomfyIohcManager::apply_slot_(uint8_t slot) {
  auto &managed = this->slots_[slot];
  auto *cover = managed.cover.get();
  if (!managed.has_record) {
    const auto defaults = default_record_(slot);
    managed.record = defaults;
    cover->set_runtime_enabled(false);
    cover->runtime_clear_receive_remote_codes();
    cover->reconfigure_storage(defaults.storage_namespace, defaults.storage_key,
                               defaults.initial_rolling_code);
    return;
  }

  auto &record = managed.record;
  cover->set_remote_code(record.node_id);
  cover->set_encryption_key(record.encryption_key);
  cover->set_open_duration(record.open_duration_ms);
  cover->set_close_duration(record.close_duration_ms);
  cover->set_my_position(record.my_position_basis_points / 10000.0f);
  cover->set_repeat_count(record.repeat_count);
  cover->reconfigure_storage(record.storage_namespace, record.storage_key,
                             record.initial_rolling_code);
  cover->runtime_clear_receive_remote_codes();
  if (record.physical_remote != 0)
    cover->add_receive_remote_code(record.physical_remote);
  const auto state = static_cast<ManagedSlotState>(record.state);
  cover->set_runtime_enabled(state == ManagedSlotState::STAGED ||
                             state == ManagedSlotState::PAIR_SENT ||
                             state == ManagedSlotState::ACTIVE);
}

bool SomfyIohcManager::complete_move_(uint8_t source_slot, uint8_t target_slot) {
  auto &source = this->slots_[source_slot];
  auto &target = this->slots_[target_slot];
  if (!source.has_record ||
      static_cast<ManagedSlotState>(source.record.state) != ManagedSlotState::ARCHIVED ||
      (source.record.flags & FLAG_MOVE_PENDING) == 0 ||
      (source.record.flags & FLAG_MOVE_TARGET_MASK) != target_slot) {
    return false;
  }

  if (target.has_record) {
    if (target.record.node_id != source.record.node_id ||
        static_cast<ManagedSlotState>(target.record.state) != ManagedSlotState::ACTIVE) {
      ESP_LOGE(TAG, "Move %u -> %u found an unrelated occupied target", source_slot,
               target_slot);
      return false;
    }
  } else {
    target.record = source.record;
    target.record.state = static_cast<uint8_t>(ManagedSlotState::ACTIVE);
    target.record.flags = 0;
    target.has_record = true;
    if (!this->save_record_(target_slot)) {
      target.has_record = false;
      target.record = default_record_(target_slot);
      return false;
    }
  }

  // Once the active destination is durable, removing the disabled source is
  // safe. If erasure fails, the archived transaction remains recoverable and
  // cannot transmit; the next boot retries cleanup.
  if (this->erase_record_(source_slot)) {
    source.has_record = false;
    source.record = default_record_(source_slot);
  } else {
    ESP_LOGW(TAG, "Move %u -> %u completed but source cleanup is pending",
             source_slot, target_slot);
  }
  this->apply_slot_(source_slot);
  this->apply_slot_(target_slot);
  return true;
}

void SomfyIohcManager::recover_pending_moves_() {
  for (uint8_t source_slot = 0; source_slot < this->slots_.size(); source_slot++) {
    const auto &record = this->slots_[source_slot].record;
    if (!this->slots_[source_slot].has_record ||
        static_cast<ManagedSlotState>(record.state) != ManagedSlotState::ARCHIVED ||
        (record.flags & FLAG_MOVE_PENDING) == 0) {
      continue;
    }
    const uint8_t target_slot = record.flags & FLAG_MOVE_TARGET_MASK;
    if (target_slot >= this->slots_.size() || target_slot == source_slot) {
      ESP_LOGE(TAG, "Slot %u contains an invalid pending-move target; quarantined",
               source_slot);
      continue;
    }
    if (this->complete_move_(source_slot, target_slot)) {
      ESP_LOGI(TAG, "Recovered interrupted slot move %u -> %u", source_slot,
               target_slot);
    } else {
      ESP_LOGE(TAG, "Could not recover interrupted slot move %u -> %u; source remains disabled",
               source_slot, target_slot);
    }
  }
}

bool SomfyIohcManager::complete_swap_(const ManagedSwapJournal &journal) {
  if (!swap_is_valid_(journal) || !this->valid_slot_(journal.first_slot) ||
      !this->valid_slot_(journal.second_slot)) {
    return false;
  }

  auto &first = this->slots_[journal.first_slot];
  auto &second = this->slots_[journal.second_slot];
  first.cover->set_runtime_enabled(false);
  second.cover->set_runtime_enabled(false);

  first.record = journal.second_record;
  first.has_record = true;
  if (!this->save_record_(journal.first_slot))
    return false;

  second.record = journal.first_record;
  second.has_record = true;
  if (!this->save_record_(journal.second_slot))
    return false;

  // The journal remains the authoritative recovery source until both swapped
  // records are durable. Removing it is the transaction commit point.
  if (!this->erase_swap_journal_())
    return false;

  this->apply_slot_(journal.first_slot);
  this->apply_slot_(journal.second_slot);
  return true;
}

bool SomfyIohcManager::recover_pending_swap_() {
  ManagedSwapJournal journal{};
  bool found = false;
  if (!this->read_swap_journal_(journal, found))
    return false;
  if (!found)
    return true;
  if (!this->complete_swap_(journal)) {
    ESP_LOGE(TAG, "Could not recover interrupted slot swap %u <-> %u; both slots remain disabled",
             journal.first_slot, journal.second_slot);
    return false;
  }
  ESP_LOGI(TAG, "Recovered interrupted slot swap %u <-> %u", journal.first_slot,
           journal.second_slot);
  return true;
}

void SomfyIohcManager::stage_slot_(int32_t requested_slot) {
  const int8_t index = requested_slot < 0
                           ? this->find_empty_slot_()
                           : (this->valid_slot_(requested_slot)
                                  ? static_cast<int8_t>(requested_slot)
                                  : -1);
  if (index < 0) {
    this->publish_status_("error", requested_slot, "invalid_or_unavailable_slot");
    return;
  }
  if (this->slots_[index].has_record) {
    this->publish_status_("error", index, "target_slot_not_empty");
    return;
  }
  auto &managed = this->slots_[index];
  managed.record = default_record_(index);
  managed.record.state = static_cast<uint8_t>(ManagedSlotState::STAGED);
  do {
    managed.record.node_id = esp_random() & 0x00FFFFFF;
  } while (managed.record.node_id == 0 ||
           managed.record.node_id == iohc::BROADCAST_ADDR ||
           this->node_id_in_use_(managed.record.node_id));
  esp_fill_random(managed.record.encryption_key, sizeof(managed.record.encryption_key));
  managed.has_record = true;
  if (!this->save_record_(index)) {
    managed.has_record = false;
    this->publish_status_("error", index, "storage_write_failed");
    return;
  }
  this->apply_slot_(index);
  this->publish_status_("staged", index);
  this->publish_backup_(index);
}

void SomfyIohcManager::start_discovery_(uint8_t slot) {
  if (static_cast<ManagedSlotState>(this->slots_[slot].record.state) !=
      ManagedSlotState::STAGED) {
    this->publish_status_("error", slot, "slot_not_staged");
    return;
  }
  if (this->discovery_slot_ >= 0 && this->discovery_slot_ != static_cast<int8_t>(slot)) {
    this->publish_status_("error", slot, "remote_discovery_busy");
    return;
  }
  this->discovery_slot_ = static_cast<int8_t>(slot);
  this->discovery_deadline_ms_ = millis() + DISCOVERY_TIMEOUT_MS;
  this->publish_status_("waiting_remote", slot);
}

void SomfyIohcManager::arm_pairing_(uint8_t slot) {
  const auto &record = this->slots_[slot].record;
  if (static_cast<ManagedSlotState>(record.state) != ManagedSlotState::STAGED) {
    this->publish_status_("error", slot, "slot_not_staged");
    return;
  }
  if (this->armed_slot_ >= 0 && this->armed_slot_ != static_cast<int8_t>(slot)) {
    this->publish_status_("error", slot, "pairing_arm_busy");
    return;
  }
  if (record.physical_remote == 0) {
    this->publish_status_("error", slot, "remote_not_captured");
    return;
  }
  this->armed_slot_ = static_cast<int8_t>(slot);
  this->arm_deadline_ms_ = millis() + PAIR_ARM_TIMEOUT_MS;
  this->publish_status_("armed", slot, "pair_within_60_seconds");
}

void SomfyIohcManager::transmit_pairing_(uint8_t slot) {
  if (this->armed_slot_ != static_cast<int8_t>(slot) ||
      static_cast<int32_t>(millis() - this->arm_deadline_ms_) >= 0) {
    this->armed_slot_ = -1;
    this->publish_status_("error", slot, "pairing_not_armed");
    return;
  }
  this->armed_slot_ = -1;  // one transmission per explicit arming
  auto &record = this->slots_[slot].record;
  // Persist the uncertain state *before* the first RF byte is emitted. If
  // power is lost during transmission, reboot must never make this identity
  // look staged and therefore eligible for an automatic second attempt.
  record.state = static_cast<uint8_t>(ManagedSlotState::PAIR_SENT);
  if (!this->save_record_(slot)) {
    record.state = static_cast<uint8_t>(ManagedSlotState::STAGED);
    this->publish_status_("error", slot, "pairing_state_save_failed");
    return;
  }
  if (!this->slots_[slot].cover->runtime_program()) {
    // The remove burst may already have left the radio. Retain PAIR_SENT and
    // require explicit human recovery instead of ever retrying automatically.
    this->publish_backup_(slot);
    this->publish_status_("error", slot, "pairing_transmit_uncertain");
    return;
  }
  this->publish_backup_(slot);
  this->publish_status_("pair_sent", slot, "confirm_motor_jog");
}

void SomfyIohcManager::confirm_pairing_(uint8_t slot) {
  auto &record = this->slots_[slot].record;
  if (static_cast<ManagedSlotState>(record.state) != ManagedSlotState::PAIR_SENT) {
    this->publish_status_("error", slot, "pairing_not_sent");
    return;
  }
  record.state = static_cast<uint8_t>(ManagedSlotState::ACTIVE);
  if (!this->save_record_(slot)) {
    record.state = static_cast<uint8_t>(ManagedSlotState::PAIR_SENT);
    this->publish_status_("error", slot, "storage_write_failed");
    return;
  }
  this->apply_slot_(slot);
  this->publish_backup_(slot);
  this->publish_status_("active", slot);
}

void SomfyIohcManager::discard_staged_(uint8_t slot) {
  auto &managed = this->slots_[slot];
  if (static_cast<ManagedSlotState>(managed.record.state) != ManagedSlotState::STAGED) {
    this->publish_status_("error", slot, "refusing_to_erase_used_identity");
    return;
  }
  if (!this->erase_record_(slot)) {
    this->publish_status_("error", slot, "storage_erase_failed");
    return;
  }
  managed.has_record = false;
  managed.record = default_record_(slot);
  this->apply_slot_(slot);
  this->publish_status_("discarded", slot);
}

void SomfyIohcManager::publish_status_(
    const char *action, int32_t slot, const char *detail, float rssi) {
  if (this->status_sensor_ == nullptr)
    return;
  this->event_counter_++;
  char state[16] = "none";
  uint32_t node = 0;
  uint32_t remote = 0;
  uint16_t next = 0;
  if (this->valid_slot_(slot)) {
    const auto &managed = this->slots_[slot];
    strlcpy(state, state_name_(static_cast<ManagedSlotState>(managed.record.state)), sizeof(state));
    node = managed.record.node_id;
    remote = managed.record.physical_remote;
    next = managed.cover->peek_next_rolling_code();
    if (next == 0)
      next = managed.record.initial_rolling_code;
  }
  char output[255];
  snprintf(output, sizeof(output),
           "{\"v\":1,\"event\":%" PRIu32 ",\"action\":\"%s\",\"slot\":%" PRId32
           ",\"state\":\"%s\",\"node\":\"0x%06" PRIX32 "\",\"remote\":\"0x%06" PRIX32
           "\",\"next\":%u,\"rssi\":%.1f,\"detail\":\"%s\"}",
           this->event_counter_, action, slot, state, node, remote, next, rssi, detail);
  this->status_sensor_->publish_state(output);
}

void SomfyIohcManager::publish_backup_(uint8_t slot) {
  if (!this->valid_slot_(slot) || !this->slots_[slot].has_record ||
      this->backup_sensor_ == nullptr)
    return;
  ManagedSlotRecord snapshot = this->slots_[slot].record;
  const uint16_t next = this->slots_[slot].cover->peek_next_rolling_code();
  if (next != 0)
    snapshot.initial_rolling_code = next;
  snapshot.checksum = record_checksum_(snapshot);
  std::string encrypted;
  if (this->encrypt_record_(snapshot, encrypted))
    this->backup_sensor_->publish_state(encrypted);
  else
    this->publish_status_("error", slot, "backup_encryption_failed");
}

void SomfyIohcManager::on_rolling_code_(uint8_t slot, uint16_t next_code) {
  if (!this->valid_slot_(slot) || !this->slots_[slot].has_record)
    return;
  this->slots_[slot].record.initial_rolling_code = next_code;
  // Do not duplicate the rolling-code NVS write. The authoritative local
  // counter was already committed by NVSRollingCodeStorage; this refreshes the
  // encrypted HA recovery snapshot for replacement hardware.
  this->publish_backup_(slot);
  this->publish_status_("backup_updated", slot);
}

void SomfyIohcManager::on_iohc_packet_(const IohcDecodedPacket &packet) {
  if (this->discovery_slot_ < 0 || packet.cmd != iohc_cmd::CMD_EXECUTE ||
      packet.data == nullptr || packet.data_len < 4)
    return;
  if (this->node_id_in_use_(packet.src_node))
    return;
  const uint8_t slot = static_cast<uint8_t>(this->discovery_slot_);
  this->discovery_slot_ = -1;
  auto &record = this->slots_[slot].record;
  record.physical_remote = packet.src_node & 0x00FFFFFF;
  if (!this->save_record_(slot)) {
    this->publish_status_("error", slot, "storage_write_failed");
    return;
  }
  this->apply_slot_(slot);
  this->publish_status_("remote_detected", slot, "", packet.rssi);
  this->publish_backup_(slot);
}

bool SomfyIohcManager::encrypt_record_(
    const ManagedSlotRecord &record, std::string &output) const {
  if (!this->has_backup_key_)
    return false;
  constexpr size_t ENVELOPE_SIZE = 1 + GCM_NONCE_SIZE + sizeof(ManagedSlotRecord) + GCM_TAG_SIZE;
  uint8_t envelope[ENVELOPE_SIZE];
  envelope[0] = BACKUP_FORMAT_VERSION;
  uint8_t *nonce = envelope + 1;
  uint8_t *ciphertext = nonce + GCM_NONCE_SIZE;
  uint8_t *tag = ciphertext + sizeof(ManagedSlotRecord);
  esp_fill_random(nonce, GCM_NONCE_SIZE);

  mbedtls_gcm_context context;
  mbedtls_gcm_init(&context);
  int result = mbedtls_gcm_setkey(
      &context, MBEDTLS_CIPHER_ID_AES, this->backup_key_, 128);
  if (result == 0) {
    result = mbedtls_gcm_crypt_and_tag(
        &context, MBEDTLS_GCM_ENCRYPT, sizeof(ManagedSlotRecord), nonce,
        GCM_NONCE_SIZE, reinterpret_cast<const uint8_t *>(GCM_AAD),
        sizeof(GCM_AAD) - 1, reinterpret_cast<const uint8_t *>(&record),
        ciphertext, GCM_TAG_SIZE, tag);
  }
  mbedtls_gcm_free(&context);
  if (result != 0)
    return false;
  output = hex_encode(envelope, sizeof(envelope));
  return output.size() <= MAX_STATE_LEN;
}

bool SomfyIohcManager::decrypt_record_(
    const std::string &input, ManagedSlotRecord &record) const {
  if (!this->has_backup_key_)
    return false;
  std::vector<uint8_t> envelope;
  if (!hex_decode(input, envelope))
    return false;
  constexpr size_t ENVELOPE_SIZE = 1 + GCM_NONCE_SIZE + sizeof(ManagedSlotRecord) + GCM_TAG_SIZE;
  if (envelope.size() != ENVELOPE_SIZE || envelope[0] != BACKUP_FORMAT_VERSION)
    return false;
  const uint8_t *nonce = envelope.data() + 1;
  const uint8_t *ciphertext = nonce + GCM_NONCE_SIZE;
  const uint8_t *tag = ciphertext + sizeof(ManagedSlotRecord);

  mbedtls_gcm_context context;
  mbedtls_gcm_init(&context);
  int result = mbedtls_gcm_setkey(
      &context, MBEDTLS_CIPHER_ID_AES, this->backup_key_, 128);
  if (result == 0) {
    result = mbedtls_gcm_auth_decrypt(
        &context, sizeof(ManagedSlotRecord), nonce, GCM_NONCE_SIZE,
        reinterpret_cast<const uint8_t *>(GCM_AAD), sizeof(GCM_AAD) - 1,
        tag, GCM_TAG_SIZE, ciphertext, reinterpret_cast<uint8_t *>(&record));
  }
  mbedtls_gcm_free(&context);
  return result == 0;
}

}  // namespace somfy
}  // namespace esphome
