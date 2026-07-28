#!/usr/bin/env bash
# Build & run the host-side test suites.
#   1. io-homecontrol protocol golden vectors (macOS only: CommonCrypto AES oracle)
#   2. RTS cover behaviour, incl. the TX -> RX state-sync loopback
# No ESPHome toolchain needed; ./stubs provides the minimal ESPHome surface.
set -euo pipefail
cd "$(dirname "$0")"

TMP="$(mktemp -d)"
CXXFLAGS=(-std=c++17 -O1 -Wall -Wextra)

echo "== io-homecontrol protocol =="
clang++ "${CXXFLAGS[@]}" \
  test_iohc_protocol.cpp ../../components/somfy/iohc_protocol.cpp \
  -o "$TMP/iohc_test"
"$TMP/iohc_test"

echo
echo "== RTS cover =="
clang++ "${CXXFLAGS[@]}" -I stubs \
  -Wno-c++20-extensions -Wno-delete-non-abstract-non-virtual-dtor \
  -DUSE_SOMFY_RTS -DUSE_SOMFY_COVER_RX \
  test_rts_cover.cpp \
  ../../components/somfy/somfy_rts.cpp \
  ../../components/somfy/somfy_hub_rts.cpp \
  ../../components/somfy/somfy_time_based_cover.cpp \
  -o "$TMP/rts_test"
"$TMP/rts_test"
