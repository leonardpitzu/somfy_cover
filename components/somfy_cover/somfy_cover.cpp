#include "esphome/core/log.h"
#include "somfy_cover.h"

namespace esphome {
namespace somfy_cover {

static const char *TAG = "somfy_cover.cover";

void SomfyCover::setup() {
  // Setup cover rolling code storage
  this->storage_ =
      new NVSRollingCodeStorage(this->storage_namespace_, this->storage_key_);

  // Attach to timebased cover controls
  automationTriggerUp_ = new Automation<>(this->get_open_trigger());
  actionTriggerUp = new SomfyCoverAction<>([=, this] { return this->open(); });
  automationTriggerUp_->add_action(actionTriggerUp);

  automationTriggerDown_ = new Automation<>(this->get_close_trigger());
  actionTriggerDown_ =
      new SomfyCoverAction<>([=, this] { return this->close(); });
  automationTriggerDown_->add_action(actionTriggerDown_);

  automationTriggerStop_ = new Automation<>(this->get_stop_trigger());
  actionTriggerStop_ =
      new SomfyCoverAction<>([=, this] { return this->stop(); });
  automationTriggerStop_->add_action(actionTriggerStop_);

  // Attach the prog button
  this->cover_prog_button_->add_on_press_callback(
      [=, this] { return this->program(); });

  // Set extra settings
  this->has_built_in_endstop_ = true;
  this->assumed_state_ = true;

  TimeBasedCover::setup();
}

void SomfyCover::loop() { TimeBasedCover::loop(); }

void SomfyCover::dump_config() { ESP_LOGCONFIG(TAG, "Somfy cover"); }

cover::CoverTraits SomfyCover::get_traits() {
  auto traits = TimeBasedCover::get_traits();
  traits.set_supports_tilt(false);

  return traits;
}

void SomfyCover::control(const cover::CoverCall &call) {
  TimeBasedCover::control(call);
}

void SomfyCover::open() {
  std::string command = "OPEN " + this->get_object_id();
  ESP_LOGI("somfy", command.c_str());
  this->send_command(Command::Up);
}

void SomfyCover::close() {
  std::string command = "CLOSE " + this->get_object_id();
  ESP_LOGI("somfy", command.c_str());
  this->send_command(Command::Down);
}

void SomfyCover::stop() {
  std::string command = "STOP " + this->get_object_id();
  ESP_LOGI("somfy", command.c_str());
  this->send_command(Command::My);
}

void SomfyCover::program() {
  std::string command = "PROG " + this->get_object_id();
  ESP_LOGI("somfy", command.c_str());
  this->send_command(Command::Prog);
}

void SomfyCover::send_command(Command command) {
  const uint16_t rollingCode = this->storage_->nextCode();
  uint8_t frame[7];
  build_frame(frame, command, rollingCode);
  remote_base::RawTimings t;
  build_timings(t, frame, 2);
  for (int i = 0; i < this->repeat_count_; i++) {
    build_timings(t, frame, 7);
  }
  auto call = this->remote_transmitter_->transmit();
  call.get_data()->set_data(t);
  call.perform();
}

void SomfyCover::build_frame(uint8_t *frame, Command command, uint16_t code) {
	const uint8_t button = static_cast<uint8_t>(command);
	frame[0] = 0xA7;          // Encryption key. Doesn't matter much
	frame[1] = button << 4;   // Which button did  you press? The 4 LSB will be the checksum
	frame[2] = code >> 8;     // Rolling code (big endian)
	frame[3] = code;          // Rolling code

  frame[4] = this->remote_code_ >> 16;  // Remote address
	frame[5] = this->remote_code_ >> 8;   // Remote address
	frame[6] = this->remote_code_;        // Remote address

	// Checksum calculation: a XOR of all the nibbles
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < 7; i++) {
		checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
	}
	checksum &= 0b1111;  // We keep the last 4 bits only

	// Checksum integration
	frame[1] |= checksum;

	// Obfuscation: a XOR of all the bytes
	for (uint8_t i = 1; i < 7; i++) {
		frame[i] ^= frame[i - 1];
	}
}

void SomfyCover::build_timings(remote_base::RawTimings & t, uint8_t *frame, uint8_t sync) {
  const int32_t SYMBOL = 640;

	if (sync == 2) {  // Only with the first frame.
		// Wake-up pulse & Silence
		send_high(t, 9415);
		send_low(t, 9565 + 80000); // was delay(80)
	}

	// Hardware sync: two sync for the first frame, seven for the following ones.
	for (int i = 0; i < sync; i++) {
		send_high(t, 4 * SYMBOL);
		send_low(t, 4 * SYMBOL);
	}

	// Software sync
	send_high(t, 4550);
	send_low(t, SYMBOL);

	// Data: bits are sent one by one, starting with the MSB.
	for (uint8_t i = 0; i < 56; i++) {
		if (((frame[i / 8] >> (7 - (i % 8))) & 1) == 1) {
			send_low(t, SYMBOL);
			send_high(t, SYMBOL);
		} else {
			send_high(t, SYMBOL);
			send_low(t, SYMBOL);
		}
	}

	// Inter-frame silence
	send_low(t, 415 + 30000); // was delay(30)
}

void SomfyCover::send_high(remote_base::RawTimings & t, int32_t durationUsecs) {
  t.push_back(static_cast<int32_t>(durationUsecs));
}

void SomfyCover::send_low(remote_base::RawTimings & t, int32_t durationUsecs) {
  t.push_back(-static_cast<int32_t>(durationUsecs));
}

} // namespace somfy_cover
} // namespace esphome
