#!/usr/bin/env bash
# Build & run the host-side test suites.
#   1. io-homecontrol protocol golden vectors (AES oracle: CommonCrypto / OpenSSL)
#   2. RX-sync animator, the dead-reckoning used to follow a physical remote
#   3. RTS cover behaviour, incl. the physical-remote -> HA state sync
#
# No ESPHome toolchain needed; ./stubs provides the minimal ESPHome surface.
# These cover what `esphome compile` cannot: a firmware build succeeding proves
# nothing about whether the covers actually track state.
set -euo pipefail
cd "$(dirname "$0")"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

CXX="${CXX:-c++}"
CXXFLAGS=(-std=c++17 -O1 -Wall -Wextra)
# The components follow ESPHome's C++20-leaning style and its non-virtual
# component destructors; neither is a defect worth failing the suite over.
COMPONENT_WARNINGS=(-Wno-c++20-extensions -Wno-delete-non-abstract-non-virtual-dtor)

# Single token, so a plain string works and sidesteps bash 3.2's unbound-variable
# error on empty array expansion under `set -u`.
if [[ "$(uname -s)" == "Darwin" ]]; then
  CRYPTO_LIBS=""  # CommonCrypto is part of libSystem
else
  CRYPTO_LIBS="-lcrypto"
fi

failed=0

run_suite() {
  local name="$1" out="$2"
  shift 2
  echo "== ${name} =="
  if "$CXX" "${CXXFLAGS[@]}" "$@" -o "$out" && "$out"; then
    echo
  else
    failed=1
    echo "!! ${name} FAILED"
    echo
  fi
}

run_suite "io-homecontrol protocol" "$TMP/iohc_protocol" \
  test_iohc_protocol.cpp \
  ../../components/somfy/iohc_protocol.cpp \
  $CRYPTO_LIBS

run_suite "RX-sync animator" "$TMP/rx_sync_animator" \
  test_rx_sync_animator.cpp

run_suite "RTS cover" "$TMP/rts_cover" \
  -I stubs "${COMPONENT_WARNINGS[@]}" \
  -DUSE_SOMFY_RTS -DUSE_SOMFY_COVER_RX \
  test_rts_cover.cpp \
  ../../components/somfy/somfy_rts.cpp \
  ../../components/somfy/somfy_hub_rts.cpp \
  ../../components/somfy/somfy_time_based_cover.cpp

if [[ "$failed" -ne 0 ]]; then
  echo "One or more C++ suites failed."
  exit 1
fi

echo "All C++ suites passed."

