#pragma once

#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/somfy/somfy_iohc.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nvs.h>

namespace esphome {
namespace somfy {

static constexpr uint8_t IOHC_MANAGER_MAX_SHUTTERS = 32;
static constexpr uint8_t IOHC_MANAGER_MAX_REMOTE_ALIASES = 32;

enum class ManagedSlotState : uint8_t {
  EMPTY = 0,
  STAGED = 1,
  PAIR_SENT = 2,
  ACTIVE = 3,
  ARCHIVED = 4,
};

// Persisted independently from each shutter's rolling-code NVS key. Never
// reorder fields without bumping version and adding an explicit migration.
struct __attribute__((packed)) ManagedSlotRecord {
  uint32_t magic;
  uint8_t version;
  uint8_t state;
  uint16_t flags;
  uint32_t node_id;
  uint8_t encryption_key[16];
  uint32_t physical_remote;
  uint32_t open_duration_ms;
  uint32_t close_duration_ms;
  uint16_t my_position_basis_points;
  uint16_t initial_rolling_code;
  uint8_t repeat_count;
  char storage_namespace[16];
  char storage_key[16];
  uint32_t checksum;
};

// Durable two-record journal used to exchange occupied slots without ever
// depending on a spare slot. Both original controller identities remain in
// this blob until their swapped destinations have been committed.
struct __attribute__((packed)) ManagedSwapJournal {
  uint32_t magic;
  uint8_t version;
  uint8_t first_slot;
  uint8_t second_slot;
  uint8_t reserved;
  ManagedSlotRecord first_record;
  ManagedSlotRecord second_record;
  uint32_t checksum;
};

// Receive-only physical-remote alias. Storing permanent controller node IDs
// instead of slot numbers makes group membership survive slot moves and swaps
// without rewriting this record. One physical group identity can target every
// managed shutter, and one shutter can appear in multiple alias records.
struct ManagedRemoteAliasRecord {
  uint32_t magic;
  uint8_t version;
  uint8_t node_count;
  uint16_t reserved;
  uint32_t remote_code;
  uint32_t node_ids[IOHC_MANAGER_MAX_SHUTTERS];
  uint32_t checksum;
};
static_assert(sizeof(ManagedRemoteAliasRecord) == 144,
              "Remote alias persistence layout changed");

class SomfyIohcManager : public Component, public api::CustomAPIDevice {
 public:
  void set_hub(SomfyIohcHub *hub) { this->hub_ = hub; }
  void set_status_sensor(text_sensor::TextSensor *sensor) { this->status_sensor_ = sensor; }
  void set_backup_sensor(text_sensor::TextSensor *sensor) { this->backup_sensor_ = sensor; }
  void set_backup_key(const char *hex_key);
  void set_max_shutters(uint8_t count) { this->max_shutters_ = count; }
  void set_cover_device_class_index(uint8_t index) { this->cover_device_class_index_ = index; }

  void add_import(uint8_t slot, uint32_t node_id, const char *encryption_key,
                  const char *storage_namespace, const char *storage_key,
                  uint16_t initial_rolling_code, uint32_t physical_remote,
                  uint32_t open_duration_ms, uint32_t close_duration_ms,
                  float my_position, uint8_t repeat_count);

  // Called from generated setup() before App.setup(), allowing the manager to
  // register a fixed pool of disabled cover entities without one YAML block
  // per shutter.
  void create_slots();

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA + 1.0f; }

 protected:
  struct Slot {
    ManagedSlotRecord record{};
    bool has_record{false};
    std::unique_ptr<SomfyIohcCover> cover;
    char entity_name[40]{};
  };

  struct RemoteAlias {
    ManagedRemoteAliasRecord record{};
    bool has_record{false};
  };

  SomfyIohcHub *hub_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
  text_sensor::TextSensor *backup_sensor_{nullptr};
  uint8_t backup_key_[16]{};
  bool has_backup_key_{false};
  uint8_t max_shutters_{IOHC_MANAGER_MAX_SHUTTERS};
  uint8_t cover_device_class_index_{0};
  std::vector<Slot> slots_;
  std::vector<std::pair<uint8_t, ManagedSlotRecord>> imports_;
  std::vector<RemoteAlias> remote_aliases_;

  nvs_handle registry_handle_{0};
  bool registry_open_{false};
  uint32_t event_counter_{0};
  uint32_t accepted_remote_command_count_{0};
  int8_t discovery_slot_{-1};
  uint32_t discovery_deadline_ms_{0};
  int8_t armed_slot_{-1};
  uint32_t arm_deadline_ms_{0};
  bool alias_discovery_active_{false};
  uint32_t alias_discovery_deadline_ms_{0};
  uint32_t captured_alias_remote_{0};
  float captured_alias_rssi_{0.0f};
  std::vector<uint32_t> alias_discovery_nodes_;

  void commission_service(std::string action, int32_t slot);
  void calibrate_service(int32_t slot, float open_seconds, float close_seconds,
                         float my_percent);
  void venetian_service(int32_t slot, bool enabled, int32_t tilt_steps,
                        bool tilt_inverted, int32_t my_tilt_step);
  void control_service(int32_t slot, std::string command, float position_percent);
  void restore_service(std::string encrypted_backup, int32_t slot);
  void move_service(int32_t slot, int32_t target_slot);
  void swap_service(int32_t slot, int32_t target_slot);
  void remote_alias_service(std::string action, std::string remote,
                            std::string slots);

  bool open_registry_();
  bool read_imports_bootstrapped_(bool &bootstrapped);
  bool mark_imports_bootstrapped_();
  bool load_record_(uint8_t slot, ManagedSlotRecord &record);
  bool save_record_(uint8_t slot);
  bool erase_record_(uint8_t slot);
  bool read_swap_journal_(ManagedSwapJournal &journal, bool &found);
  bool save_swap_journal_(ManagedSwapJournal &journal);
  bool erase_swap_journal_();
  bool load_remote_aliases_();
  bool load_remote_alias_record_(uint8_t index,
                                 ManagedRemoteAliasRecord &record,
                                 bool &found);
  bool save_remote_alias_record_(uint8_t index);
  bool erase_remote_alias_record_(uint8_t index);
  static ManagedSlotRecord default_record_(uint8_t slot);
  static uint32_t record_checksum_(const ManagedSlotRecord &record);
  static uint32_t swap_checksum_(const ManagedSwapJournal &journal);
  static uint32_t remote_alias_checksum_(
      const ManagedRemoteAliasRecord &record);
  static bool record_is_valid_(const ManagedSlotRecord &record);
  static bool swap_is_valid_(const ManagedSwapJournal &journal);
  static bool remote_alias_is_valid_(
      const ManagedRemoteAliasRecord &record);
  static const char *state_name_(ManagedSlotState state);
  static bool parse_hex_key_(const char *value, uint8_t key[16]);

  bool valid_slot_(int32_t slot) const;
  int8_t find_empty_slot_() const;
  bool node_id_in_use_(uint32_t node_id) const;
  bool parse_remote_code_(const std::string &value, uint32_t &remote) const;
  bool resolve_alias_slots_(const std::string &value,
                            std::vector<uint32_t> &node_ids) const;
  std::string alias_slots_for_nodes_(
      const std::vector<uint32_t> &node_ids) const;
  int8_t find_remote_alias_(uint32_t remote) const;
  int8_t find_empty_remote_alias_() const;
  bool remote_alias_contains_node_(const ManagedRemoteAliasRecord &record,
                                   uint32_t node_id) const;
  void apply_slot_remote_filters_(uint8_t slot);
  void apply_all_slots_();
  void start_alias_discovery_(const std::string &slots);
  void set_remote_alias_(uint32_t remote,
                         const std::vector<uint32_t> &node_ids);
  void remove_remote_alias_(uint32_t remote);
  void apply_slot_(uint8_t slot);
  void stage_slot_(int32_t requested_slot);
  bool complete_move_(uint8_t source_slot, uint8_t target_slot);
  void recover_pending_moves_();
  bool complete_swap_(const ManagedSwapJournal &journal);
  bool recover_pending_swap_();
  void start_discovery_(uint8_t slot);
  void arm_pairing_(uint8_t slot);
  void transmit_pairing_(uint8_t slot);
  void confirm_pairing_(uint8_t slot);
  void discard_staged_(uint8_t slot);
  void publish_status_(const char *action, int32_t slot, const char *detail = "",
                       float rssi = 0.0f, uint32_t remote_override = 0);
  void publish_rx_stats_(int32_t slot);
  void publish_backup_(uint8_t slot);
  void on_rolling_code_(uint8_t slot, uint16_t next_code);
  void on_iohc_packet_(const IohcDecodedPacket &packet);

  bool encrypt_record_(const ManagedSlotRecord &record, std::string &output) const;
  bool decrypt_record_(const std::string &input, ManagedSlotRecord &record) const;
};

}  // namespace somfy
}  // namespace esphome
