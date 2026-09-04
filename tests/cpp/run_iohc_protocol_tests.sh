#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
test_bin="$(mktemp /private/tmp/iohc-control-test.XXXXXX)"
trap 'rm -f "$test_bin"' EXIT

if [[ "$(uname -s)" == "Darwin" ]]; then
  c++ -std=c++17 -O1 -Wall -Wextra \
    "$script_dir/test_iohc_protocol.cpp" \
    "$repo_root/components/somfy/iohc_protocol.cpp" \
    -o "$test_bin"
else
  c++ -std=c++17 -O1 -Wall -Wextra \
    "$script_dir/test_iohc_protocol.cpp" \
    "$repo_root/components/somfy/iohc_protocol.cpp" \
    -o "$test_bin" -lcrypto
fi
"$test_bin"
