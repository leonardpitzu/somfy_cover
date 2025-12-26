#pragma once

#include "esphome/core/gpio.h"
#include "esphome/components/button/button.h"
#include "esphome/components/time_based/time_based_cover.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "esphome/components/remote_receiver/remote_receiver.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <vector>
#include <string>
#include <cinttypes>
#include <algorithm>

// Libraries for SomfyRemote
#include "NVSRollingCodeStorage.h"

#define COVER_OPEN 1.0f
#define COVER_CLOSED 0.0f

namespace esphome {
namespace somfy_cover {

// Helper class to attach cover functions to the time based cover triggers
template <typename... Ts> class SomfyCoverAction : public Action<Ts...> {
public:
  // The function to be called when the action plays.
  std::function<void(Ts...)> callback;

  explicit SomfyCoverAction(std::function<void(Ts...)> callback)
      : callback(callback) {}

  void play(Ts... x) override {
    if (callback)
      callback(x...);
  }
};

enum class Command : uint8_t {
	My      = 0x1,
	Up      = 0x2,
	MyUp    = 0x3,
	Down    = 0x4,
	MyDown  = 0x5,
	UpDown  = 0x6,
	Prog    = 0x8,
	SunFlag = 0x9,
	Flag    = 0xA
};

class SomfyCover : public time_based::TimeBasedCover, public remote_base::RemoteReceiverListener {
public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  bool on_receive(remote_base::RemoteReceiveData data) override;

  void set_remote_transmitter(remote_transmitter::RemoteTransmitterComponent *t) {
    this->remote_transmitter_ = t;
  }

  void set_remote_receiver(remote_receiver::RemoteReceiverComponent *r) { this->remote_receiver_ = r; }
  void add_receive_remote_code(uint32_t code) { this->receive_remote_codes_.push_back(code); }
  void set_log_text_sensor(text_sensor::TextSensor *ts) { this->log_text_sensor_ = ts; }

  // Set somfy cover button and value
  void set_prog_button(button::Button *cover_prog_button) {
    this->cover_prog_button_ = cover_prog_button;
  }
  void set_remote_code(uint32_t remote_code_) {
    this->remote_code_ = remote_code_;
  }
  void set_storage_namespace(const char *storage_namespace_) {
    this->storage_namespace_ = storage_namespace_;
  }
  void set_storage_key(const char *storage_key_) {
    this->storage_key_ = storage_key_;
  }

  void set_repeat_count(int repeat_count_) {
    this->repeat_count_ = repeat_count_;
  }

  void set_emitter_pin(InternalGPIOPin *emitter) {
    this->emitter_pin_ = emitter;
  }

  cover::CoverTraits get_traits() override;

protected:
  void control(const cover::CoverCall &call) override;

  // Set via the ESPHome yaml
  remote_transmitter::RemoteTransmitterComponent *remote_transmitter_{nullptr};
  button::Button *cover_prog_button_{nullptr};
  uint32_t remote_code_{0};
  const char *storage_namespace_;
  const char *storage_key_;
  int repeat_count_{4};
  InternalGPIOPin *emitter_pin_{nullptr};

  // Optional receiver path (for keeping HA in sync when physical remotes are used)
  remote_receiver::RemoteReceiverComponent *remote_receiver_{nullptr};
  std::vector<uint32_t> receive_remote_codes_;
  text_sensor::TextSensor *log_text_sensor_{nullptr};
  uint32_t last_rx_ms_{0};

  // RX-derived UI sync (no TX). We simulate time-based movement when a physical remote is used.
  bool rx_sync_active_{false};
  cover::CoverOperation rx_operation_{cover::COVER_OPERATION_IDLE};
  uint32_t rx_start_ms_{0};
  float rx_start_pos_{0.0f};
  uint32_t rx_last_publish_ms_{0};

  // Set via the constructor
  NVSRollingCodeStorage *storage_;

  void open();
  void close();
  void stop();
  void program();

  // Create automations to attach the cover control functions
  Automation<> *automationTriggerUp_;
  SomfyCoverAction<> *actionTriggerUp;
  Automation<> *automationTriggerDown_;
  SomfyCoverAction<> *actionTriggerDown_;
  Automation<> *automationTriggerStop_;
  SomfyCoverAction<> *actionTriggerStop_;

  
  bool is_allowed_remote_(uint32_t code) const;
  void send_command(Command command);

  bool decode_frame_(const remote_base::RawTimings &data, uint32_t &remote_code, uint16_t &rolling_code, Command &command);
  static const char *command_to_string_(Command cmd);

  void build_frame(uint8_t *frame, Command command, uint16_t code);
  void build_timings(remote_base::RawTimings & t, uint8_t *frame, uint8_t sync);

  void send_high(remote_base::RawTimings & t, int32_t durationInMicroseconds);
  void send_low(remote_base::RawTimings & t, int32_t durationInMicroseconds);

};

} // namespace somfy_cover
} // namespace esphome
