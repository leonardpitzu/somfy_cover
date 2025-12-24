#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "somfy_cover.h"

namespace esphome {
namespace somfy_cover {

static const char *TAG = "somfy_cover.cover";

static const uint32_t SOMFY_SYMBOL_US = 640;
static const uint32_t SOMFY_SOFTWARE_SYNC_MARK_US = 4550;

const char *SomfyCover::command_to_string_(Command cmd) {
  switch (cmd) {
    case Command::My:
      return "MY";
    case Command::Up:
      return "UP";
    case Command::Down:
      return "DOWN";
    case Command::MyUp:
      return "MY_UP";
    case Command::MyDown:
      return "MY_DOWN";
    case Command::UpDown:
      return "UP_DOWN";
    case Command::Prog:
      return "PROG";
    case Command::SunFlag:
      return "SUN_FLAG";
    case Command::Flag:
      return "FLAG";
    default:
      return "UNKNOWN";
  }
}

bool SomfyCover::decode_frame_(const remote_base::RawTimings &data, uint32_t &remote_code, uint16_t &rolling_code,
                               Command &command) {
  // Look for the (software sync) mark + space pair, then decode 56 bits of Manchester data.
  // We keep this intentionally simple: it is meant for "learn remote code once, paste into YAML".
  const int n = static_cast<int>(data.size());
  if (n < 10)
    return false;

  auto approx = [](int32_t v, uint32_t target, uint32_t tol) -> bool {
    const uint32_t av = static_cast<uint32_t>(std::abs(v));
    return av >= (target > tol ? target - tol : 0) && av <= target + tol;
  };

  // Tolerance: Somfy timings are not ultra precise; accept ~30%.
  const uint32_t tol_sync = SOMFY_SOFTWARE_SYNC_MARK_US / 3;
  const uint32_t tol_sym = SOMFY_SYMBOL_US / 2;

  int start = -1;
  for (int i = 0; i + 1 < n; i++) {
    if (data[i] > 0 && approx(data[i], SOMFY_SOFTWARE_SYNC_MARK_US, tol_sync) && data[i + 1] < 0 &&
        approx(data[i + 1], SOMFY_SYMBOL_US, tol_sym)) {
      start = i + 2;
      break;
    }
  }
  if (start < 0)
    return false;

  if (start + 56 * 2 > n)
    return false;

  uint8_t raw[7]{0};
  for (int bit = 0; bit < 56; bit++) {
    const int32_t first = data[start + bit * 2];
    const int32_t second = data[start + bit * 2 + 1];

    // Validate symbol lengths (best-effort).
    if (!approx(first, SOMFY_SYMBOL_US, tol_sym) || !approx(second, SOMFY_SYMBOL_US, tol_sym))
      return false;

    const uint8_t b = (first < 0) ? 1 : 0;  // 1 => space then mark, 0 => mark then space
    raw[bit / 8] |= (b << (7 - (bit % 8)));
  }

  // De-obfuscate (reverse of: frame[i] ^= frame[i-1] for i=1..6)
  uint8_t frame[7]{0};
  frame[0] = raw[0];
  for (int i = 1; i < 7; i++) {
    frame[i] = raw[i] ^ raw[i - 1];
  }

  // Validate checksum: XOR of all nibbles, keep low 4 bits.
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
  }
  checksum &= 0x0F;
  if ((frame[1] & 0x0F) != checksum)
    return false;

  const uint8_t cmd_nibble = (frame[1] >> 4) & 0x0F;
  command = static_cast<Command>(cmd_nibble);
  rolling_code = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
  remote_code = (static_cast<uint32_t>(frame[4]) << 16) | (static_cast<uint32_t>(frame[5]) << 8) | frame[6];

  return true;
}

bool SomfyCover::on_receive(remote_base::RemoteReceiveData data) {
  if (this->log_codes_) {
    const auto &raw = data.get_raw_data();
    ESP_LOGD(TAG, "RX callback for '%s': raw_len=%u", this->name_.c_str(), (unsigned) raw.size());
    if (!raw.empty()) {
      std::string s;
      const size_t n = std::min<size_t>(raw.size(), 20);
      for (size_t i = 0; i < n; i++) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", (int) raw[i]);
        s += buf;
        if (i + 1 < n) s += ",";
      }
      ESP_LOGD(TAG, "RX first timings: %s", s.c_str());
    }
  }
  // If nothing is configured, don't spend cycles decoding.
  if (!this->log_codes_ && this->receive_remote_codes_.empty())
    return false;

  // Basic de-duplication: Somfy remotes send repeats, and we may receive multiple frames per press.
  const uint32_t now = millis();
  if (now - this->last_rx_ms_ < 150) {
    return false;
  }

  uint32_t remote_code = 0;
  uint16_t rolling = 0;
  Command cmd = Command::My;

  if (!this->decode_frame_(data.get_raw_data(), remote_code, rolling, cmd))
    return false;

  this->last_rx_ms_ = now;

  const bool is_known_remote =
      std::find(this->receive_remote_codes_.begin(), this->receive_remote_codes_.end(), remote_code) !=
      this->receive_remote_codes_.end();

  if (this->log_codes_) {
    ESP_LOGI(TAG, "RX: remote_code=0x%06" PRIX32 " cmd=%s rolling=0x%04" PRIX16 "%s", remote_code,
             this->command_to_string_(cmd), rolling, is_known_remote ? " (known)" : "");
  } else if (!is_known_remote) {
    // Not logging and not known => ignore.
    return false;
  }

  if (this->log_text_sensor_ != nullptr) {
    char buf[96];
    snprintf(buf, sizeof(buf), "0x%06" PRIX32 " %s 0x%04" PRIX16, remote_code, this->command_to_string_(cmd), rolling);
    this->log_text_sensor_->publish_state(buf);
  }

  if (!is_known_remote)
    return true;

  // Keep HA in sync (simple: assume full open/closed on UP/DOWN, stop on MY/UP_DOWN).
  switch (cmd) {
    case Command::Up:
    case Command::MyUp:
      this->position = COVER_OPEN;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state();
      break;
    case Command::Down:
    case Command::MyDown:
      this->position = COVER_CLOSED;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state();
      break;
    case Command::My:
    case Command::UpDown:
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state();
      break;
    default:
      break;
  }

  return true;
}


void SomfyCover::setup() {
  // Setup cover rolling code storage
  this->storage_ =
      new NVSRollingCodeStorage(this->storage_namespace_, this->storage_key_);


  // Optional receiver support
  if (this->remote_receiver_ != nullptr) {
    ESP_LOGD(TAG, "RX: registering receiver listener for cover '%s'", this->name_.c_str());
    this->remote_receiver_->register_listener(this);
  } else {
    ESP_LOGD(TAG, "RX: no remote_receiver configured for cover '%s'", this->name_.c_str());
  }

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
