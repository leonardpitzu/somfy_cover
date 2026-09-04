#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC

#include "esphome/core/component.h"
#include "esphome/components/cc1101/cc1101.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace esphome {
namespace somfy {

// io-homecontrol protocol constants
namespace iohc {

// 1W radio parameters
static constexpr float FREQUENCY_1W = 868.95e6f;
static constexpr float SYMBOL_RATE = 38400.0f;
static constexpr float FSK_DEVIATION = 19200.0f;
static constexpr float FILTER_BW = 100000.0f;

// 2W radio parameters - 3 channel frequency hopping
static constexpr float FREQUENCY_2W_CH0 = 868.25e6f;
static constexpr float FREQUENCY_2W_CH1 = 868.95e6f;
static constexpr float FREQUENCY_2W_CH2 = 869.85e6f;
static constexpr float FREQUENCY_2W[] = {FREQUENCY_2W_CH0, FREQUENCY_2W_CH1, FREQUENCY_2W_CH2};
static constexpr uint32_t CHANNEL_DWELL_US = 2700;

// Logical sync bytes are 0xFF 0x33. On air they are UART-encoded, so the CC1101
// hardware sync word is programmed to iohc_proto::PHY_HW_SYNC1/0 (0x57FD), a
// preamble-tail-aligned window of the encoded sequence — see iohc_protocol.h.

// Fixed-length RX capture windows (raw on-air bytes collected after a sync
// match). The hardware-validated 1W receiver uses 60 bytes so a complete frame
// can be recovered from the remote's repeated burst despite CC1101 sync/FIFO
// alignment. Shortening it to the encoded application-frame size prevented
// the FIFO from completing on real Situo presses. Both windows must stay <= 64
// bytes (the CC1101 FIFO depth); the software UART decoder and ctrl0 length
// field discard trailing bytes.
static constexpr uint8_t RX_FIFO_WINDOW_1W = 60;
static constexpr uint8_t RX_FIFO_WINDOW_2W = 60;

// Broadcast address
static constexpr uint32_t BROADCAST_ADDR = 0x00003F;

// 2W protocol timing
static constexpr uint32_t SESSION_TIMEOUT_MS = 3000;
static constexpr uint8_t SESSION_MAX_RETRIES = 2;

// 2W command IDs
static constexpr uint8_t CMD_EXECUTE = 0x00;
static constexpr uint8_t CMD_PRIVATE_ACK = 0x21;
static constexpr uint8_t CMD_ASK_CHALLENGE = 0x31;
static constexpr uint8_t CMD_KEY_TRANSFER = 0x32;
static constexpr uint8_t CMD_KEY_TRANSFER_ACK = 0x33;
static constexpr uint8_t CMD_LAUNCH_KEY_TRANSFER = 0x38;
static constexpr uint8_t CMD_CHALLENGE_REQUEST = 0x3C;
static constexpr uint8_t CMD_CHALLENGE_RESPONSE = 0x3D;
static constexpr uint8_t CMD_STATUS = 0xFE;

// 2W frame control byte flags
static constexpr uint8_t CTRL1_2W = 0x00;       // no Start/End framing bits

}  // namespace iohc

// Decoded CRC-valid iohc RX packet. The frame/data pointers remain valid only
// for the duration of the registered RX callbacks; consumers that need them
// later must copy the bytes.
struct IohcDecodedPacket {
  uint8_t ctrl0{0};
  uint8_t ctrl1{0};
  uint32_t dest_node{0};
  uint32_t src_node{0};
  uint8_t cmd{0};
  const uint8_t *data{nullptr};
  size_t data_len{0};
  const uint8_t *frame{nullptr};
  size_t frame_len{0};
  float rssi{0};
  uint8_t lqi{0};
};

// Callback for iohc RX packet notifications
using IohcRxCallback = std::function<void(const IohcDecodedPacket &pkt)>;

// 2W session completion callback: success flag + optional response data
using Session2WCallback = std::function<void(bool success, const IohcDecodedPacket *response)>;

// CRC-16-KERMIT helper (shared between hub and devices)
uint16_t crc16_kermit(const uint8_t *data, size_t len);

// AES-128 ECB helper (shared between hub and devices)
void aes128_ecb_encrypt(const uint8_t key[16], const uint8_t plaintext[16], uint8_t ciphertext[16]);

// 2W challenge response computation
void compute_2w_response(const uint8_t key[16], const uint8_t *frame_data, size_t frame_len,
                         const uint8_t challenge[6], uint8_t response[6]);

// 2W state machine states
enum class Session2WState : uint8_t {
  IDLE,
  CMD_SENT,         // Command sent, waiting for 0x3C challenge
  CHALLENGE_RCVD,   // Challenge received, response being sent
  WAIT_STATUS,      // Waiting for status/ack from actuator
  COMPLETE,         // Session done
  FAILED,           // Session timed out or error
};

// A pending 2W session (one per command exchange)
struct Session2W {
  Session2WState state{Session2WState::IDLE};
  uint32_t src_node{0};
  uint32_t dest_node{0};
  uint8_t cmd{0};
  std::vector<uint8_t> cmd_data;
  std::vector<uint8_t> frame_payload;  // cmd+data (authenticated MAC input)
  uint8_t key[16]{};
  uint8_t challenge[6]{};
  uint32_t started_ms{0};
  uint8_t retries{0};
  Session2WCallback callback;
};

/// iohc radio hub — owns the CC1101, provides TX/RX and radio configuration.
/// Devices register for RX callbacks and call transmit_packet() for TX.
class SomfyIohcHub : public Component,
                     public cc1101::CC1101Listener {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration
  void set_cc1101(cc1101::CC1101Component *cc1101) { this->cc1101_ = cc1101; }
  void set_frequency_1w(float frequency) { this->frequency_1w_ = frequency; }

  // TX: transmit a raw packet (frame bytes including CRC) with repeats (1W mode)
  void transmit_packet(const std::vector<uint8_t> &frame, uint8_t repeat_count);

  // 2W TX: send a command with challenge/response authentication
  void send_2w_command(uint32_t src_node, uint32_t dest_node, uint8_t cmd,
                       const uint8_t *data, size_t data_len,
                       const uint8_t key[16], Session2WCallback callback);

  // After TX, return to RX mode
  void begin_rx();

  // Radio mode configuration
  void configure_radio_1w();
  void configure_radio_2w(uint8_t channel);

  // 2W listening control
  void start_2w_listen();
  void stop_2w_listen();

  // RX: register a device to receive decoded packets
  void register_rx_callback(IohcRxCallback callback) {
    this->rx_callbacks_.push_back(std::move(callback));
  }

  // Monotonic boot-local counters for reception diagnostics. A raw packet has
  // passed the CC1101 sync/FIFO layer; a valid frame also passed UART decode,
  // length validation, and protocol CRC.
  uint32_t get_rx_raw_packet_count() const { return this->rx_raw_packet_count_; }
  uint32_t get_rx_valid_frame_count() const { return this->rx_valid_frame_count_; }
  float get_last_raw_frequency_offset() const { return this->last_raw_frequency_offset_; }
  float get_last_raw_rssi() const { return this->last_raw_rssi_; }
  float get_last_valid_rssi() const { return this->last_valid_rssi_; }

  // CC1101Listener interface
  void on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi, uint8_t lqi) override;

 protected:
  cc1101::CC1101Component *cc1101_{nullptr};
  float frequency_1w_{iohc::FREQUENCY_1W};

  // Reusable UART-codec buffers, so TX and RX do not allocate per packet.
  std::vector<uint8_t> tx_payload_;
  std::vector<uint8_t> rx_frame_;

  // 2W hopping state
  bool listening_2w_{false};
  uint8_t current_2w_channel_{0};
  uint32_t last_hop_us_{0};

  // RX dispatch
  std::vector<IohcRxCallback> rx_callbacks_;
  uint32_t rx_raw_packet_count_{0};
  uint32_t rx_valid_frame_count_{0};
  float last_raw_frequency_offset_{0.0f};
  float last_raw_rssi_{0.0f};
  float last_valid_rssi_{0.0f};

  // 2W session management
  Session2W session_;
  void session_loop_();
  void handle_2w_packet_(const IohcDecodedPacket &pkt);
  void send_2w_frame_(uint32_t src, uint32_t dest, uint8_t cmd,
                      const uint8_t *data, size_t data_len);
  std::vector<uint8_t> build_2w_frame_(uint32_t src, uint32_t dest, uint8_t cmd,
                                        const uint8_t *data, size_t data_len);
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
