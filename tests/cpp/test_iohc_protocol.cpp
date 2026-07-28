// Host-side golden-vector tests for the io-homecontrol protocol core.
//
// Builds and runs on the developer machine (no ESP32/ESPHome/mbedTLS needed).
// AES-128-ECB comes from the platform crypto library as a trusted oracle, so
// the test validates our framing/CRC/checksum/IV/MAC math, not an AES impl.
//
// Build & run:  tests/cpp/run.sh   (or see that script for the compiler line)
//
// Golden vectors are sourced from and cross-checked against the reference
// implementation Velocet/iown-homecontrol (scripts/Iown-ioCrypto.py + aes.py),
// which is the authoritative description of the link-layer crypto.

#include "../../components/somfy/iohc_protocol.h"

#ifdef __APPLE__
#include <CommonCrypto/CommonCryptor.h>
#else
#include <openssl/evp.h>
#endif

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace esphome::somfy;

// --- AES oracle (AES-128-ECB, single block, no padding) --------------------
#ifdef __APPLE__
static void aes128_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
  size_t moved = 0;
  CCCryptorStatus s = CCCrypt(kCCEncrypt, kCCAlgorithmAES, kCCOptionECBMode, key,
                              kCCKeySizeAES128, nullptr, in, 16, out, 16, &moved);
  if (s != kCCSuccess || moved != 16) {
    std::fprintf(stderr, "AES oracle failure: status=%d moved=%zu\n", (int) s, moved);
    std::exit(2);
  }
}
#else
static void aes128_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
  int len = 0;
  bool ok = ctx != nullptr && EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, key, nullptr) == 1;
  if (ok) {
    EVP_CIPHER_CTX_set_padding(ctx, 0);
    ok = EVP_EncryptUpdate(ctx, out, &len, in, 16) == 1 && len == 16;
  }
  EVP_CIPHER_CTX_free(ctx);
  if (!ok) {
    std::fprintf(stderr, "AES oracle failure (OpenSSL)\n");
    std::exit(2);
  }
}
#endif

// --- tiny test harness -----------------------------------------------------
static int g_failures = 0;
static int g_checks = 0;

static std::vector<uint8_t> hx(const std::string &hex) {
  std::vector<uint8_t> out;
  std::string clean;
  for (char c : hex)
    if (c != ' ' && c != '\n' && c != '\t')
      clean.push_back(c);
  for (size_t i = 0; i + 1 < clean.size(); i += 2)
    out.push_back(static_cast<uint8_t>(std::stoul(clean.substr(i, 2), nullptr, 16)));
  return out;
}

static std::string to_hex(const uint8_t *d, size_t n) {
  static const char *H = "0123456789ABCDEF";
  std::string s;
  for (size_t i = 0; i < n; i++) {
    s.push_back(H[d[i] >> 4]);
    s.push_back(H[d[i] & 0xF]);
  }
  return s;
}

static void check_bytes(const char *name, const std::vector<uint8_t> &expected,
                        const uint8_t *actual, size_t n) {
  g_checks++;
  bool ok = (expected.size() == n) && (memcmp(expected.data(), actual, n) == 0);
  if (ok) {
    std::printf("  PASS  %-28s %s\n", name, to_hex(actual, n).c_str());
  } else {
    g_failures++;
    std::printf("  FAIL  %-28s got=%s want=%s\n", name, to_hex(actual, n).c_str(),
                to_hex(expected.data(), expected.size()).c_str());
  }
}

static void check_u16(const char *name, uint16_t expected, uint16_t actual) {
  g_checks++;
  if (expected == actual) {
    std::printf("  PASS  %-28s 0x%04X\n", name, actual);
  } else {
    g_failures++;
    std::printf("  FAIL  %-28s got=0x%04X want=0x%04X\n", name, actual, expected);
  }
}

int main() {
  std::printf("io-homecontrol protocol golden-vector tests\n");

  const std::vector<uint8_t> TRANSFER_KEY =
      hx("34C3466ED88F4E8E16AA473949884373");
  // Device/stack key used in the iown-homecontrol linklayer worked examples.
  const std::vector<uint8_t> DEVICE_KEY =
      hx("01020304050607080910111213141516");

  // 1) CRC-16 (poly 0x8408). Golden value 0x525F over the 25-byte sequence,
  //    and a full frame including its own CRC must check to 0x0000.
  {
    auto v = hx("F8 00 00 00 3F 1A 38 0B 00 01 61 00 00 80 D8 05 00 02 A6 24 22 2E 8B A3 51");
    uint16_t crc = iohc_proto::crc16(v.data(), v.size());
    check_u16("crc16 vector", 0x525F, crc);
    // append LSB-first and re-run -> 0x0000
    v.push_back(static_cast<uint8_t>(crc & 0xFF));
    v.push_back(static_cast<uint8_t>(crc >> 8));
    check_u16("crc16 frame-residue", 0x0000, iohc_proto::crc16(v.data(), v.size()));
  }

  // 1b) A complete, verified io-homecontrol 1W frame (the reference 0x30
  //     key-push frame, ctrl0..CRC with CRC appended LSB-first) must check to
  //     0x0000 over the whole frame, proving byte order + CRC end-to-end.
  {
    auto f = hx("fc0000003fabcdef307e60491f976adf653db0ed785e49a2010201123419e81ec43d5e9bf2");
    check_u16("crc16 frame 0x0000", 0x0000, iohc_proto::crc16(f.data(), f.size()));
  }

  // 2) Rolling checksum.
  {
    uint8_t c1, c2;
    auto a = hx("31");
    iohc_proto::rolling_checksum(a.data(), a.size(), c1, c2);
    uint8_t got[2] = {c1, c2};
    check_bytes("checksum(31)", hx("0062"), got, 2);

    auto b = hx("00 01 43 D2 00 00 00");  // cmd||data for an EXECUTE STOP
    iohc_proto::rolling_checksum(b.data(), b.size(), c1, c2);
    uint8_t got2[2] = {c1, c2};
    check_bytes("checksum(EXECUTE)", hx("0500"), got2, 2);
  }

  // 3) 1W IV construction (linklayer worked example).
  {
    auto payload = hx("00 01 43 D2 00 00 00");  // cmd||data
    uint8_t iv[16];
    iohc_proto::build_iv_1w(payload.data(), payload.size(), 0x0599, iv);
    check_bytes("iv_1w", hx("000143D2000000550500059955555555"), iv, 16);
  }

  // 4) 2W IV construction (challenge example).
  {
    auto payload = hx("31");
    auto challenge = hx("123456789ABC");
    uint8_t iv[16];
    iohc_proto::build_iv_2w(payload.data(), payload.size(), challenge.data(), iv);
    check_bytes("iv_2w", hx("31555555555555550062123456789ABC"), iv, 16);
  }

  // 5) 2W challenge response (linklayer push example, key-transfer 0x32 frame).
  //    payload = cmd||data of the 0x32 frame; challenge 123456789ABC.
  {
    auto payload = hx("32 102E49A16D3B69726F3192CF17534AD9");
    auto challenge = hx("123456789ABC");
    uint8_t iv[16];
    iohc_proto::build_iv_2w(payload.data(), payload.size(), challenge.data(), iv);
    uint8_t mac[6];
    iohc_proto::compute_mac(aes128_ecb, DEVICE_KEY.data(), iv, mac);
    check_bytes("2w response", hx("C0251949AFFD"), mac, 6);
  }

  // 6) 1W controller-key obfuscation for the 0x30 frame (node ABCDEF).
  {
    uint8_t enc[16];
    iohc_proto::obfuscate_key_1w(aes128_ecb, TRANSFER_KEY.data(), 0xABCDEF,
                                 DEVICE_KEY.data(), enc);
    check_bytes("0x30 key obfuscation", hx("7E60491F976ADF653DB0ED785E49A201"), enc, 16);

    // 7) 1W MAC for that 0x30 frame: the MAC is computed over
    //    payload = cmd(0x30) || enc_key(16) ONLY (17 bytes). The trailing
    //    manufacturer(0x02)/0x01 bytes are appended to the on-air frame but are
    //    NOT authenticated. Sequence 0x1234, device's own key.
    std::vector<uint8_t> payload;
    payload.push_back(0x30);
    payload.insert(payload.end(), enc, enc + 16);
    uint8_t iv[16];
    iohc_proto::build_iv_1w(payload.data(), payload.size(), 0x1234, iv);
    uint8_t mac[6];
    iohc_proto::compute_mac(aes128_ecb, DEVICE_KEY.data(), iv, mac);
    check_bytes("0x30 1W MAC", hx("19E81EC43D5E"), mac, 6);
  }

  // 8) Physical-layer UART-8N1 codec.
  {
    // 8a) Per-byte UART framing must match the documented sync derivation.
    //     A local encoder (start 0, 8 data bits LSB-first, stop 1, MSB-first
    //     bit packing) over the sync bytes 0xFF 0x33 yields the 20-bit sequence
    //     0111111111 0110011001. The first 16 bits are the hardware sync word
    //     0x7FD9 and the next 4 bits (0b1001) are the encoded-sync tail.
    auto uart_pack = [](const std::vector<uint8_t> &bytes) {
      std::vector<uint8_t> out;
      size_t count = 0;
      auto put = [&](uint8_t bit) {
        if (count % 8 == 0)
          out.push_back(0);
        if (bit & 1)
          out.back() |= static_cast<uint8_t>(1u << (7 - (count % 8)));
        count++;
      };
      for (uint8_t b : bytes) {
        put(0);
        for (uint8_t k = 0; k < 8; k++)
          put((b >> k) & 1);
        put(1);
      }
      return out;
    };
    auto sync = uart_pack(hx("FF33"));
    check_bytes("phy sync FF33 encoding", hx("7FD990"), sync.data(), sync.size());
    check_u16("phy hw sync word",
              static_cast<uint16_t>((iohc_proto::PHY_HW_SYNC1 << 8) | iohc_proto::PHY_HW_SYNC0),
              0x7FD9);

    // 8b) Golden FIFO payload for a single 0x00 logical byte: tail 1001 +
    //     start 0 + 00000000 + stop 1, padded with idle 1s -> 0x90 0x07.
    std::vector<uint8_t> enc0;
    uint8_t zero = 0x00;
    iohc_proto::uart_encode(&zero, 1, enc0);
    check_bytes("phy encode(00)", hx("9007"), enc0.data(), enc0.size());

    // 8c) Encode -> decode round-trip on a complete, verified 1W frame
    //     (ctrl0 .. CRC) must recover the exact logical bytes.
    auto frame = hx("fc0000003fabcdef307e60491f976adf653db0ed785e49a2010201123419e81ec43d5e9bf2");
    std::vector<uint8_t> packed;
    iohc_proto::uart_encode(frame.data(), frame.size(), packed);
    std::vector<uint8_t> decoded;
    size_t n = iohc_proto::uart_decode(packed.data(), packed.size(), decoded);
    check_bytes("phy round-trip", frame, decoded.data(), n);
  }

  std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}
