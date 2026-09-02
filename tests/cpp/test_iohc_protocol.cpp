#include "../../components/somfy/iohc_protocol.h"
#include "../../components/somfy/rx_sync_animator.h"

#ifdef __APPLE__
#include <CommonCrypto/CommonCryptor.h>
#else
#include <openssl/evp.h>
#endif

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace esphome::somfy;

namespace {

int failures = 0;

std::vector<uint8_t> hex(const std::string &text) {
  std::vector<uint8_t> result;
  std::string clean;
  for (const char c : text) {
    if (c != ' ' && c != '\n' && c != '\t')
      clean.push_back(c);
  }
  for (size_t i = 0; i + 1 < clean.size(); i += 2)
    result.push_back(static_cast<uint8_t>(std::stoul(clean.substr(i, 2), nullptr, 16)));
  return result;
}

void aes128_ecb(const uint8_t key[16], const uint8_t input[16], uint8_t output[16]) {
#ifdef __APPLE__
  size_t moved = 0;
  const CCCryptorStatus status = CCCrypt(kCCEncrypt, kCCAlgorithmAES, kCCOptionECBMode,
                                         key, kCCKeySizeAES128, nullptr, input, 16,
                                         output, 16, &moved);
  if (status != kCCSuccess || moved != 16) {
    std::fprintf(stderr, "AES oracle failed: status=%d moved=%zu\n", status, moved);
    std::exit(2);
  }
#else
  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
  int moved = 0;
  int final_moved = 0;
  if (ctx == nullptr || EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, key, nullptr) != 1 ||
      EVP_CIPHER_CTX_set_padding(ctx, 0) != 1 ||
      EVP_EncryptUpdate(ctx, output, &moved, input, 16) != 1 ||
      EVP_EncryptFinal_ex(ctx, output + moved, &final_moved) != 1 ||
      moved + final_moved != 16) {
    std::fprintf(stderr, "OpenSSL AES oracle failed\n");
    EVP_CIPHER_CTX_free(ctx);
    std::exit(2);
  }
  EVP_CIPHER_CTX_free(ctx);
#endif
}

void expect(bool condition, const char *name) {
  std::printf("%s %s\n", condition ? "PASS" : "FAIL", name);
  if (!condition)
    failures++;
}

}  // namespace

int main() {
  const auto transfer_key = hex("34C3466ED88F4E8E16AA473949884373");
  const auto device_key = hex("01020304050607080910111213141516");

  uint8_t encrypted_key[16];
  iohc_proto::obfuscate_key_1w(aes128_ecb, transfer_key.data(), 0xABCDEF,
                               device_key.data(), encrypted_key);
  std::vector<uint8_t> key_frame;
  expect(iohc_proto::build_key_transfer_frame_1w(0xABCDEF, 0x00003F,
                                                  encrypted_key, 0x02, 0x01,
                                                  0x1234, 0x00, key_frame),
         "build hardware-validated 0x30 key-transfer frame");
  const auto expected_key_frame = hex(
      "FC 00 00003F ABCDEF 30 7E60491F976ADF653DB0ED785E49A201 "
      "02 01 1234 3911");
  expect(key_frame == expected_key_frame,
         "0x30 bytes match the 31-byte captured format");
  expect(key_frame.size() == 31 && (key_frame[0] & 0x1F) == 28,
         "0x30 declared size and total length are correct");
  expect(iohc_proto::crc16(key_frame.data(), key_frame.size()) == 0,
         "0x30 CRC residue is zero");

  const uint8_t stop_data[6] = {0x01, 0x43, 0xD2, 0x00, 0x00, 0x00};
  std::vector<uint8_t> stop_frame;
  expect(iohc_proto::build_frame_1w(aes128_ecb, device_key.data(), 0xABCDEF,
                                    0x00003F, 0x00, stop_data, sizeof(stop_data),
                                    sizeof(stop_data), 0x0599, stop_frame),
         "build captured-format STOP frame");
  expect(stop_frame.size() == 25 && stop_frame[0] == 0xF6,
         "STOP logical length and ctrl0 are correct");
  expect(std::memcmp(stop_frame.data() + 9, stop_data, sizeof(stop_data)) == 0,
         "STOP data is 01 43 D2 00 00 00");
  expect(stop_frame[15] == 0x05 && stop_frame[16] == 0x99,
         "STOP sequence is in the captured position");
  uint16_t received_sequence = 0;
  expect(iohc_proto::extract_sequence_1w(stop_frame.data() + 9, stop_frame.size() - 11,
                                         received_sequence) &&
             received_sequence == 0x0599,
         "extract rolling sequence from decoded STOP data");
  expect(!iohc_proto::extract_sequence_1w(stop_frame.data() + 9, 7, received_sequence),
         "reject decoded 1W data too short to contain sequence and MAC");
  expect(iohc_proto::crc16(stop_frame.data(), stop_frame.size()) == 0,
         "STOP CRC residue is zero");

  expect(iohc_proto::gesture_terminal_matches_prefix(
             0x61620C, 0x0599, true, 0x61620C, 0x059A, true),
         "adjacent terminal sequence completes a D200 prefix");
  expect(!iohc_proto::gesture_terminal_matches_prefix(
             0x61620C, 0x0599, true, 0x61620C, 0x059B, true),
         "non-adjacent terminal sequence does not consume a prefix");
  expect(!iohc_proto::gesture_terminal_matches_prefix(
             0x61620C, 0x0599, true, 0x61620D, 0x059A, true),
         "terminal from another remote does not consume a prefix");
  expect(iohc_proto::gesture_terminal_matches_prefix(
             0xFF61620C, 0, false, 0x0061620C, 0x059A, true),
         "sequence-less evidence falls back to masked same-remote correlation");
  expect(iohc_proto::gesture_terminal_matches_prefix(
             0x61620C, 0xFFFF, true, 0x61620C, 0x0000, true),
         "terminal correlation handles the uint16 sequence boundary");

  std::vector<uint8_t> encoded;
  iohc_proto::uart_encode(stop_frame.data(), stop_frame.size(), encoded);
  expect(!encoded.empty() && encoded[0] == 0x99,
         "CC1101 FIFO payload begins with the validated 0x99 sync residue");
  expect(encoded.size() == 33,
         "25-byte logical frame encodes to 33 FIFO bytes");
  std::vector<uint8_t> decoded;
  iohc_proto::uart_decode(encoded.data(), encoded.size(), decoded);
  expect(decoded == stop_frame, "STOP UART-8N1 encode/decode round-trip");

  // A real idle STOP/MY short press continues with two authenticated cmd=0x20
  // frames. These payloads were captured from the paired kitchen-door remote.
  const uint8_t my_press_data[8] = {0x02, 0xFF, 0x01, 0x43, 0x02, 0x0C, 0x00, 0x00};
  std::vector<uint8_t> my_press_frame;
  expect(iohc_proto::build_frame_1w(aes128_ecb, device_key.data(), 0xABCDEF,
                                    0x00003F, 0x20, my_press_data, sizeof(my_press_data),
                                    sizeof(my_press_data), 0x059A, my_press_frame),
         "build captured-format MY press-event frame");
  expect(my_press_frame.size() == 27 && my_press_frame[0] == 0xF8 && my_press_frame[8] == 0x20,
         "MY press-event command and logical length are correct");
  expect(std::memcmp(my_press_frame.data() + 9, my_press_data, sizeof(my_press_data)) == 0,
         "MY press-event data is 02 FF 01 43 02 0C 00 00");
  expect(my_press_frame[17] == 0x05 && my_press_frame[18] == 0x9A &&
             iohc_proto::crc16(my_press_frame.data(), my_press_frame.size()) == 0,
         "MY press-event sequence and CRC are correct");

  const uint8_t my_release_data[8] = {0x02, 0xFF, 0x01, 0x43, 0x02, 0x05, 0xFF, 0x00};
  std::vector<uint8_t> my_release_frame;
  expect(iohc_proto::build_frame_1w(aes128_ecb, device_key.data(), 0xABCDEF,
                                    0x00003F, 0x20, my_release_data, sizeof(my_release_data),
                                    sizeof(my_release_data), 0x059B, my_release_frame),
         "build captured-format MY release-event frame");
  expect(my_release_frame.size() == 27 && my_release_frame[0] == 0xF8 && my_release_frame[8] == 0x20,
         "MY release-event command and logical length are correct");
  expect(std::memcmp(my_release_frame.data() + 9, my_release_data, sizeof(my_release_data)) == 0,
         "MY release-event data is 02 FF 01 43 02 05 FF 00");
  expect(my_release_frame[17] == 0x05 && my_release_frame[18] == 0x9B &&
             iohc_proto::crc16(my_release_frame.data(), my_release_frame.size()) == 0,
         "MY release-event sequence and CRC are correct");
  expect(std::memcmp(my_press_frame.data() + 19, my_release_frame.data() + 19, 6) != 0,
         "MY press and release events have independently authenticated MACs");

  std::vector<uint8_t> my_encoded;
  iohc_proto::uart_encode(my_press_frame.data(), my_press_frame.size(), my_encoded);
  expect(my_encoded.size() == 35 && my_encoded[0] == 0x99,
         "27-byte MY event encodes to the expected CC1101 FIFO length");

  iohc_proto::RxBurstDeduplicator rx_dedup;
  expect(!rx_dedup.is_duplicate(1000, 0x61620C, 0xD200, 0x1000, true, 1500),
         "first physical-remote frame is a new event");
  expect(rx_dedup.is_duplicate(1050, 0x61620C, 0xD200, 0x1000, true, 1500),
         "same rolling sequence is collapsed as an RF burst copy");
  expect(!rx_dedup.is_duplicate(1100, 0x61620C, 0xD200, 0x1001, true, 1500),
         "new rolling sequence preserves a rapid repeated button press");
  expect(rx_dedup.is_duplicate(1150, 0x61620C, 0xD200, 0x1001, true, 1500),
         "copies of the rapid repeated press are still collapsed");
  expect(!rx_dedup.is_duplicate(2651, 0x61620C, 0xD200, 0x1001, true, 1500),
         "same sequence outside the burst window is treated as a new event");

  iohc_proto::RxBurstDeduplicator fallback_dedup;
  expect(!fallback_dedup.is_duplicate(1000, 0x61620C, 0x0000, 0, false, 1500) &&
             fallback_dedup.is_duplicate(1050, 0x61620C, 0x0000, 0, false, 1500),
         "frames without a sequence retain timed main-parameter deduplication");

  RxSyncAnimator my_opening;
  my_opening.start_to(0.42f, 0.0f, 1000);
  auto my_open_half = my_opening.update(3100, 10000);
  expect(my_opening.active() && my_opening.opening() &&
             std::fabs(my_open_half.position - 0.21f) < 0.001f,
         "MY animation uses proportional opening duration");
  auto my_open_done = my_opening.update(5200, 10000);
  expect(my_open_done.finished && std::fabs(my_open_done.position - 0.42f) < 0.001f,
         "MY opening animation finishes at configured percentage");

  RxSyncAnimator my_closing;
  my_closing.start_to(0.42f, 1.0f, 2000);
  auto my_close_half = my_closing.update(4900, 10000);
  expect(my_closing.active() && !my_closing.opening() &&
             std::fabs(my_close_half.position - 0.71f) < 0.001f,
         "MY animation uses proportional closing duration");
  auto my_close_done = my_closing.update(7800, 10000);
  expect(my_close_done.finished && std::fabs(my_close_done.position - 0.42f) < 0.001f,
         "MY closing animation finishes at configured percentage");

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
