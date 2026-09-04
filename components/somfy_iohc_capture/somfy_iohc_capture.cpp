#include "somfy_iohc_capture.h"

#include "esphome/core/log.h"

#include <cinttypes>
#include <cstdio>
#include <string>

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.iohc.capture";

static std::string compact_hex(const uint8_t *data, size_t len) {
  static constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  if (data == nullptr || len == 0)
    return "";

  std::string output;
  output.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    output.push_back(HEX_DIGITS[data[i] >> 4]);
    output.push_back(HEX_DIGITS[data[i] & 0x0F]);
  }
  return output;
}

void SomfyIohcCapture::setup() {
  if (this->hub_ == nullptr) {
    ESP_LOGE(TAG, "Cannot start: Somfy iohc hub is missing");
    this->mark_failed();
    return;
  }
  this->hub_->register_rx_callback([this](const IohcDecodedPacket &packet) {
    this->on_iohc_packet_(packet);
  });
  ESP_LOGI(TAG, "Receive-only raw frame capture ready");
}

void SomfyIohcCapture::dump_config() {
  ESP_LOGCONFIG(TAG, "Somfy IOHC receive-only capture:");
  ESP_LOGCONFIG(TAG, "  Capture sensor: %s", this->capture_sensor_ != nullptr ? "configured" : "MISSING");
  if (this->filter_remote_)
    ESP_LOGCONFIG(TAG, "  Remote filter: 0x%06" PRIX32, this->remote_code_);
  else
    ESP_LOGCONFIG(TAG, "  Remote filter: none (capturing every CRC-valid frame)");
}

void SomfyIohcCapture::on_iohc_packet_(const IohcDecodedPacket &packet) {
  if (this->filter_remote_ && packet.src_node != this->remote_code_)
    return;

  const std::string data_hex = compact_hex(packet.data, packet.data_len);
  const std::string frame_hex = compact_hex(packet.frame, packet.frame_len);
  const uint32_t event = ++this->event_counter_;

  // The event number prevents Home Assistant from collapsing repeated copies
  // or repeated button presses with otherwise identical application bytes.
  char metadata[176];
  snprintf(metadata, sizeof(metadata),
           "{\"e\":%" PRIu32 ",\"src\":\"0x%06" PRIX32 "\",\"dst\":\"0x%06" PRIX32
           "\",\"cmd\":\"0x%02X\",\"c0\":\"0x%02X\",\"c1\":\"0x%02X\",\"rssi\":%.1f,\"lqi\":%u,",
           event, packet.src_node, packet.dest_node, packet.cmd, packet.ctrl0, packet.ctrl1,
           packet.rssi, packet.lqi);

  std::string state(metadata);
  state.reserve(state.size() + data_hex.size() + frame_hex.size() + 24);
  state.append("\"data\":\"");
  state.append(data_hex);
  state.append("\",\"frame\":\"");
  state.append(frame_hex);
  state.append("\"}");

  ESP_LOGI(TAG,
           "event=%" PRIu32 " src=0x%06" PRIX32 " dst=0x%06" PRIX32
           " cmd=0x%02X c0=0x%02X c1=0x%02X rssi=%.1f lqi=%u data=[%s] frame=[%s]",
           event, packet.src_node, packet.dest_node, packet.cmd, packet.ctrl0, packet.ctrl1,
           packet.rssi, packet.lqi, data_hex.c_str(), frame_hex.c_str());

  if (this->capture_sensor_ != nullptr)
    this->capture_sensor_->publish_state(state);
}

}  // namespace somfy
}  // namespace esphome
