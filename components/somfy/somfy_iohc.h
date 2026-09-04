#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC

#include "iohc_protocol.h"
#include "somfy_hub_iohc.h"
#include "NVSRollingCodeStorage.h"
#include "esphome/components/button/button.h"
#include "rx_sync_animator.h"
#include "somfy_time_based_cover.h"
#include "esphome/core/component.h"
#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
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
// Companion event frames emitted by real Somfy 1W remotes for every short
// button press. Hardware captures prove that idle MY needs these after its
// extended D200 EXECUTE frame; an ordinary short D200 alone only stops motion.
static constexpr uint8_t CMD_BUTTON_EVENT = 0x20;
static constexpr uint8_t CMD_WRITE_PRIVATE = 0x30;
static constexpr uint8_t CMD_REMOVE_CONTROLLER = 0x39;

// Main Parameters for CMD_EXECUTE
static constexpr uint16_t MP_OPEN = 0x0000;
static constexpr uint16_t MP_CLOSE = 0xC800;
static constexpr uint16_t MP_STOP = 0xD200;
static constexpr uint16_t MP_MY = 0xD800;

// CMD_BUTTON_EVENT payload action byte (captured at data offset 4).
static constexpr uint8_t BUTTON_ACTION_OPEN = 0x00;
static constexpr uint8_t BUTTON_ACTION_CLOSE = 0x01;
static constexpr uint8_t BUTTON_ACTION_STOP_MY = 0x02;
// Hardware-verified Situo 5 Variation wheel detents. A detent is a D200
// EXECUTE frame with direction-specific FP bytes followed by one of these
// private actions; unlike an ordinary button press, no release was observed.
static constexpr uint8_t BUTTON_ACTION_TILT_CLOCKWISE = 0x0D;
static constexpr uint8_t BUTTON_ACTION_TILT_COUNTERCLOCKWISE = 0x0E;

static constexpr uint32_t TILT_EVENT_DELAY_MS = 25;
static constexpr uint32_t TILT_NEXT_STEP_DELAY_MS = 100;
// A received wheel detent can expose an ordinary-looking D200 frame before
// its direction-bearing private event. Hold D200 briefly so the UI receives
// either STOP/MY or tilt, never both. Hardware measured a 57 ms gap; the wider
// window tolerates missed early burst copies without making a real stop feel
// materially delayed in Home Assistant.
static constexpr uint32_t RX_GESTURE_CORRELATION_MS = 350;
// The Situo Variation wheel encodes signed travel around 0xCCE8 in the FP
// word of the leading D200 EXECUTE. Hardware captures in both directions show
// one wheel unit per 0x0046; the following private 0x0D/0x0E event remains the
// authoritative direction indicator.
static constexpr uint16_t TILT_MAGNITUDE_CENTER = 0xCCE8;
static constexpr uint16_t TILT_MAGNITUDE_UNIT = 0x0046;
// A native one-unit wheel frame is the motor's minimum one-step gesture. For
// larger rolls, hardware trials show approximately two encoded magnitude
// units per effective calibrated slat step. Keep the exact minimum gesture,
// and apply this scale to compound TX and RX estimates.
static constexpr uint8_t TILT_MAGNITUDE_UNITS_PER_STEP = 2;
// Hardware trials found raw magnitude 4 reliably moves two effective steps in
// both directions. Larger synthetic magnitudes become non-linear and raw 20
// was rejected entirely, so long moves are deliberately split into exact,
// smoother two-step chunks instead of depending on unsupported large rolls.
static constexpr uint16_t TILT_MAX_GESTURE_STEPS = 2;
// Endpoint requests deliberately drive beyond the calibrated number of
// effective detents. Motors ignore extra commands at their mechanical limit,
// making 0% and 100% reliable resynchronisation points.
static constexpr uint8_t TILT_ENDPOINT_MARGIN_STEPS = 2;

// A dedicated HA MY action first sends a lone D200 stop, waits for the motor to
// settle, and only then reproduces the native three-frame MY press. This gives
// MY unambiguous "go to favourite" semantics regardless of the motor's state.
static constexpr uint32_t MY_PRESTOP_SETTLE_MS = 500;
// These delays are measured from the end of one local six-copy transmit burst.
// They reproduce the approximately 300 ms EXECUTE-to-event and 100 ms
// event-to-event start times of the physical remote without blocking ESPHome's
// main loop.
static constexpr uint32_t MY_EVENT_DELAY_MS = 220;
static constexpr uint32_t MY_RELEASE_DELAY_MS = 25;

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
  void set_storage_key(const char *key) { this->storage_key_ = key == nullptr ? "" : key; }
  void set_storage_namespace(const char *ns) { this->storage_namespace_ = ns == nullptr ? "" : ns; }
  void set_initial_rolling_code(uint16_t code) { this->initial_rolling_code_ = code; }
  void set_repeat_count(int count) { this->repeat_count_ = count; }
  void set_encryption_key(const char *hex_key);
  void set_encryption_key(const uint8_t key[16]);
  void set_mode(IohcMode mode) { this->mode_ = mode; }
  void set_target_node(uint32_t node) { this->target_node_ = node & 0x00FFFFFF; }
  void set_venetian(bool enabled, uint8_t tilt_steps = 12,
                    bool tilt_inverted = false, uint8_t my_tilt_step = 6);
  bool is_venetian() const { return this->venetian_; }
  cover::CoverTraits get_traits() override;

  // Runtime-manager hooks. Ordinary YAML-defined covers remain enabled by
  // default and continue to use the same paths. Managed slot covers are born
  // disabled, then receive their persisted identity before RF is enabled.
  void set_runtime_enabled(bool enabled);
  bool is_runtime_enabled() const { return this->runtime_enabled_; }
  void reconfigure_storage(const char *ns, const char *key, uint16_t initial_code);
  void set_rolling_code_callback(std::function<void(uint16_t)> callback) {
    this->rolling_code_callback_ = std::move(callback);
  }
  void set_remote_command_callback(
      std::function<void(uint16_t, uint32_t, float, uint8_t)> callback) {
    this->remote_command_callback_ = std::move(callback);
  }
  void set_relay_frame_callback(
      std::function<void(uint16_t, const std::vector<uint8_t> &, uint8_t)> callback) {
    this->relay_frame_callback_ = std::move(callback);
  }
  // A manager-owned cover is decoded centrally by SomfyIohcManager. This
  // avoids one physical packet advancing the estimator twice while allowing
  // YAML-defined standalone covers to retain their original RX path.
  void set_manager_rx_owned(bool owned) { this->manager_rx_owned_ = owned; }
  void set_my_sequence_complete_callback(std::function<void()> callback) {
    this->my_sequence_complete_callback_ = std::move(callback);
  }
  void set_tilt_sequence_complete_callback(std::function<void()> callback) {
    this->tilt_sequence_complete_callback_ = std::move(callback);
  }
  uint16_t peek_next_rolling_code() const;
  bool runtime_seed_rolling_code(uint16_t next_code);
  void runtime_clear_receive_remote_codes();
  bool runtime_program();
  void runtime_open();
  void runtime_close();
  void runtime_stop();
  void runtime_my();
  void runtime_set_position(float position);
  void runtime_set_tilt(float tilt);
  void runtime_tilt_step(bool clockwise);
  void runtime_stop_tilt() { this->cancel_1w_tilt_sequence(); }
  // Estimator-only hooks for cross-bridge observations. They deliberately do
  // not enter control(), build a frame, or consume a rolling code.
  void runtime_observe_command(uint16_t main_param);
  void runtime_observe_tilt(bool clockwise, uint8_t steps);
  void runtime_setup() { this->call_setup(); }
  void runtime_loop() { this->call(); }

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
  // Manager-created slots are configured from records that may be temporary
  // stack values while slots are reset or moved. Own these strings so setup()
  // never observes a dangling NVS namespace/key pointer.
  std::string storage_key_;
  std::string storage_namespace_;
  uint16_t initial_rolling_code_{1};
  int repeat_count_{iohc_cmd::TX_REPEAT_COUNT};

  // Native motor favourite (MY) position, represented in ESPHome's 0..1
  // cover scale. The value is a configured estimate because 1W has no actual
  // position feedback.
  float my_position_{0.5f};
  bool has_my_position_{false};

  // Venetian tilt is step-based on 1W remotes. tilt_steps_ is the calibrated
  // number of wheel detents between the two slat endpoints. The public cover
  // tilt value remains ESPHome's standard 0..1 scale and is restored by the
  // cover base class.
  bool venetian_{false};
  bool tilt_inverted_{false};
  uint8_t tilt_steps_{12};
  // Physical clockwise detents from the counterclockwise endpoint. Keeping
  // this installation-facing value separate from the logical HA percentage
  // makes direction inversion unambiguous.
  uint8_t my_tilt_step_{6};
  int16_t tilt_steps_remaining_{0};
  int8_t tilt_direction_{0};
  float tilt_target_{1.0f};

  // Protocol mode
  IohcMode mode_{IohcMode::MODE_1W};

  // Encryption key (system key for 2W, controller key for 1W)
  uint8_t encryption_key_[16]{};
  bool has_custom_key_{false};
  bool runtime_enabled_{true};

  // Rolling code storage
  std::unique_ptr<NVSRollingCodeStorage> storage_;
  std::function<void(uint16_t)> rolling_code_callback_;
  std::function<void(uint16_t, uint32_t, float, uint8_t)>
      remote_command_callback_;
  std::function<void(uint16_t, const std::vector<uint8_t> &, uint8_t)>
      relay_frame_callback_;
  std::function<void()> my_sequence_complete_callback_;
  std::function<void()> tilt_sequence_complete_callback_;
  bool my_sequence_active_{false};
  bool tilt_sequence_active_{false};
  bool manager_rx_owned_{false};

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
  bool program();
  uint16_t next_rolling_code_();

  // 1W Protocol (per-device: uses device key + rolling code)
  bool send_1w_command(uint16_t main_param);
  bool send_1w_my_execute();
  bool send_1w_button_event(uint8_t action, bool released);
  bool send_1w_my_sequence();
  void cancel_1w_my_sequence();
  void finish_1w_my_sequence_();
  bool send_1w_tilt_execute(bool clockwise, uint16_t steps);
  void set_tilt_target_(float target);
  float logical_tilt_for_physical_step_(uint8_t physical_step) const;
  void set_lift_tilt_(bool opening, bool publish = true);
  void set_my_tilt_(bool publish = true);
  void start_tilt_steps_(int16_t steps, int8_t logical_direction, float target);
  void send_next_tilt_step_();
  void cancel_1w_tilt_sequence();
  void finish_1w_tilt_sequence_();
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

  // Repeat-burst suppression: a physical remote transmits the same frame
  // several times back-to-back (and the CC1101 hands us each copy separately).
  // Sequence-aware matching collapses RF copies without swallowing a rapid new
  // press of the same button.
  iohc_proto::RxBurstDeduplicator rx_deduplicator_;
  uint32_t rx_event_counter_{0};

  // Physical-remote UI animation state.
  RxSyncAnimator rx_sync_;
  bool my_tilt_pending_{false};
  bool pending_rx_stop_{false};
  uint32_t pending_rx_stop_remote_{0};
  uint32_t pending_rx_stop_deadline_ms_{0};
  float pending_rx_stop_rssi_{0.0f};
  uint16_t pending_rx_stop_sequence_{0};
  bool pending_rx_stop_has_sequence_{false};
  uint8_t pending_rx_tilt_steps_{1};

  void start_rx_sync(cover::CoverOperation op);
  void start_rx_sync_to(float target_position);
  void stop_rx_sync();

  bool is_allowed_remote_(uint32_t code) const;
  // Decode the MainParameter from a CMD_EXECUTE packet (foreign remote command).
  static bool decode_execute_param_(const IohcDecodedPacket &pkt, uint16_t &main_param);
  static bool decode_button_action_(const IohcDecodedPacket &pkt, uint8_t &action);
  static bool decode_tilt_execute_steps_(const IohcDecodedPacket &pkt,
                                         uint8_t &steps);
  void stage_rx_stop_(uint32_t remote, float rssi, uint16_t sequence,
                      bool has_sequence, uint8_t tilt_steps);
  uint8_t consume_rx_tilt_steps_for_(uint32_t remote);
  void clear_pending_rx_stop_();
  void flush_pending_rx_stop_();
  void emit_rx_command_(uint16_t main_param, uint32_t remote, float rssi,
                        uint16_t sequence, bool has_sequence);
  // Drive the HA UI animation in response to a recognised foreign command.
  void handle_rx_command_(uint16_t main_param);
  void handle_rx_tilt_steps_(bool clockwise, uint8_t steps);
#endif
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
