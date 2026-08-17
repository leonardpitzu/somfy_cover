#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC

#include "somfy_hub_iohc.h"
#include "NVSRollingCodeStorage.h"
#include "esphome/components/button/button.h"
#include "rx_sync_animator.h"
#include "somfy_time_based_cover.h"
#include "esphome/core/component.h"
#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#ifdef USE_SOMFY_IOHC_RX
namespace esphome {
namespace text_sensor {
class TextSensor;
}
}  // namespace esphome
#endif

namespace esphome {
namespace somfy {

// Transfer key (public, used for 1W encryption and 2W key exchange)
namespace iohc_keys {
static constexpr uint8_t TRANSFER_KEY[16] = {
    0x34, 0xC3, 0x46, 0x6E, 0xD8, 0x8F, 0x4E, 0x8E,
    0x16, 0xAA, 0x47, 0x39, 0x49, 0x88, 0x43, 0x73
};
}  // namespace iohc_keys

// Commands
namespace iohc_cmd {
static constexpr uint8_t CMD_EXECUTE = 0x00;
static constexpr uint8_t CMD_WRITE_PRIVATE = 0x30;
static constexpr uint8_t CMD_REMOVE_CONTROLLER = 0x39;

// Main Parameters for CMD_EXECUTE
static constexpr uint16_t MP_OPEN = 0x0000;
static constexpr uint16_t MP_CLOSE = 0xC800;
static constexpr uint16_t MP_STOP = 0xD200;
static constexpr uint16_t MP_MY = 0xD800;

// Originator IDs
static constexpr uint8_t ORIGINATOR_USER = 0x01;
static constexpr uint8_t ORIGINATOR_RAIN = 0x02;
static constexpr uint8_t ORIGINATOR_TIMER = 0x03;
static constexpr uint8_t ORIGINATOR_SECURITY = 0x08;

// ACEI (Access Control & Encryption Info). The physical Situo 1W remote used
// for this installation consistently transmits 0x43 in EXECUTE frames.
static constexpr uint8_t ACEI_DEFAULT = 0x43;   // 1W
static constexpr uint8_t ACEI_2W = 0x61;        // 2W

static constexpr uint8_t TX_REPEAT_COUNT = 4;
static constexpr uint8_t PAIR_REPEAT_COUNT = 4;
}  // namespace iohc_cmd

// Protocol mode
enum class IohcMode : uint8_t {
  MODE_1W,   // One-way (broadcast, HMAC-authenticated)
  MODE_2W,   // Two-way (unicast, challenge/response authenticated)
};

/// Action wrapper that runs a plain callback when a cover trigger fires.
template<typename... Ts> class SomfyIohcAction : public Action<Ts...> {
 public:
  explicit SomfyIohcAction(std::function<void()> callback) : callback_(std::move(callback)) {}
  void play(Ts... x) override {
    if (this->callback_)
      this->callback_();
  }

 protected:
  std::function<void()> callback_;
};

class SomfyIohcCover : public SomfyTimeBasedCover {
 public:
  void setup() override;
#ifdef USE_SOMFY_IOHC_RX
  void loop() override;
#endif
  void dump_config() override;

  // Configuration setters
  void set_hub(SomfyIohcHub *hub) { this->hub_ = hub; }
  void set_prog_button(button::Button *btn) { this->prog_button_ = btn; }
  void set_my_button(button::Button *btn) { this->my_button_ = btn; }
  void set_my_position(float position) {
    this->my_position_ = position;
    this->has_my_position_ = true;
  }
  void set_remote_code(uint32_t code) { this->node_id_ = code & 0x00FFFFFF; }
  void set_storage_key(const char *key) { this->storage_key_ = key; }
  void set_storage_namespace(const char *ns) { this->storage_namespace_ = ns; }
  void set_initial_rolling_code(uint16_t code) { this->initial_rolling_code_ = code; }
  void set_repeat_count(int count) { this->repeat_count_ = count; }
  void set_encryption_key(const char *hex_key);
  void set_mode(IohcMode mode) { this->mode_ = mode; }
  void set_target_node(uint32_t node) { this->target_node_ = node & 0x00FFFFFF; }

#ifdef USE_SOMFY_IOHC_RX
  // RX state-sync configuration (mirrors the RTS allowed_remotes/detected_remote
  // feature). Codes are the 3-byte node IDs of physical io-homecontrol remotes.
  void add_receive_remote_code(uint32_t code) {
    code &= 0x00FFFFFF;
    auto it = std::lower_bound(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), code);
    if (it == this->receive_remote_codes_.end() || *it != code)
      this->receive_remote_codes_.insert(it, code);
  }
  void set_log_text_sensor(text_sensor::TextSensor *ts) { this->log_text_sensor_ = ts; }
#endif

 protected:
  void control(const cover::CoverCall &call) override;

  // Hub reference (owns radio)
  SomfyIohcHub *hub_{nullptr};
  button::Button *prog_button_{nullptr};
  button::Button *my_button_{nullptr};

  // Per-device identity
  uint32_t node_id_{0};
  uint32_t target_node_{0};  // 2W: destination actuator address
  const char *storage_key_{nullptr};
  const char *storage_namespace_{nullptr};
  uint16_t initial_rolling_code_{1};
  int repeat_count_{iohc_cmd::TX_REPEAT_COUNT};

  // Native motor favourite (MY) position, represented in ESPHome's 0..1
  // cover scale. The value is a configured estimate because 1W has no actual
  // position feedback.
  float my_position_{0.5f};
  bool has_my_position_{false};

  // Protocol mode
  IohcMode mode_{IohcMode::MODE_1W};

  // Encryption key (system key for 2W, controller key for 1W)
  uint8_t encryption_key_[16]{};
  bool has_custom_key_{false};

  // Rolling code storage
  std::unique_ptr<NVSRollingCodeStorage> storage_;

  // Cover trigger wiring (open/close/stop -> radio commands)
  std::unique_ptr<Automation<>> open_automation_;
  std::unique_ptr<Automation<>> close_automation_;
  std::unique_ptr<Automation<>> stop_automation_;
  std::unique_ptr<SomfyIohcAction<>> open_action_;
  std::unique_ptr<SomfyIohcAction<>> close_action_;
  std::unique_ptr<SomfyIohcAction<>> stop_action_;

  // Commands
  void open();
  void close();
  void stop();
  void my();
  void program();

  // 1W Protocol (per-device: uses device key + rolling code)
  void send_1w_command(uint16_t main_param);
  // Build a complete ordinary 1W frame. The MAC authenticates
  // cmd || data[0..auth_len); auth_len defaults to the full data length.
  // Pairing's special no-MAC 0x30 frame uses the protocol helper directly.
  std::vector<uint8_t> build_1w_frame(uint8_t cmd, const uint8_t *data, size_t data_len,
                                      uint32_t dest_node, size_t auth_len = SIZE_MAX);

  // 2W Protocol (uses challenge/response via hub session)
  void send_2w_command(uint16_t main_param);
  void on_2w_result_(bool success, const IohcDecodedPacket *response);

  // RX handler
  void on_iohc_packet_(const IohcDecodedPacket &pkt);

#ifdef USE_SOMFY_IOHC_RX
  // RX state-sync: keep HA in sync with physical io-homecontrol remotes.
  std::vector<uint32_t> receive_remote_codes_;
  text_sensor::TextSensor *log_text_sensor_{nullptr};

  // Repeat-burst suppression: a physical remote transmits the same frame several
  // times back-to-back (and the CC1101 hands us each copy separately). Collapse
  // identical (src, main_param) pairs seen within a short window.
  uint32_t rx_dedup_src_{0};
  uint16_t rx_dedup_param_{0};
  uint32_t rx_dedup_ms_{0};
  bool rx_dedup_valid_{false};

  // Physical-remote UI animation state.
  RxSyncAnimator rx_sync_;

  void start_rx_sync(cover::CoverOperation op);
  void start_rx_sync_to(float target_position);
  void stop_rx_sync();

  bool is_allowed_remote_(uint32_t code) const;
  // Decode the MainParameter from a CMD_EXECUTE packet (foreign remote command).
  static bool decode_execute_param_(const IohcDecodedPacket &pkt, uint16_t &main_param);
  // True if this (src, main_param) is a duplicate of the previous one inside the
  // dedup window (i.e. part of the remote's repeat burst).
  bool rx_is_duplicate_(uint32_t src, uint16_t main_param);
  // Drive the HA UI animation in response to a recognised foreign command.
  void handle_rx_command_(uint16_t main_param);
#endif
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
