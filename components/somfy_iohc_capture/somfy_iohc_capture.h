#pragma once

#include "esphome/components/somfy/somfy_hub_iohc.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

#include <cstdint>

namespace esphome {
namespace somfy {

// Receive-only diagnostic used while reverse-engineering physical remote
// behaviour. It registers an RX callback but owns no transmitter methods,
// controller key, rolling code or pairing action.
class SomfyIohcCapture : public Component {
 public:
  void set_hub(SomfyIohcHub *hub) { this->hub_ = hub; }
  void set_capture_sensor(text_sensor::TextSensor *sensor) { this->capture_sensor_ = sensor; }
  void set_remote_code(uint32_t code) {
    this->remote_code_ = code & 0x00FFFFFF;
    this->filter_remote_ = true;
  }

  void setup() override;
  void dump_config() override;

 protected:
  SomfyIohcHub *hub_{nullptr};
  text_sensor::TextSensor *capture_sensor_{nullptr};
  uint32_t remote_code_{0};
  uint32_t event_counter_{0};
  bool filter_remote_{false};

  void on_iohc_packet_(const IohcDecodedPacket &packet);
};

}  // namespace somfy
}  // namespace esphome
