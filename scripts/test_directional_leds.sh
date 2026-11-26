#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_DIR="$(mktemp -d)"
trap 'rm -rf "${TEST_DIR}"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I "${ROOT_DIR}" "${ROOT_DIR}/tests/test_directional_leds.cpp" -o "${TEST_DIR}/directional_leds"
"${TEST_DIR}/directional_leds"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I "${ROOT_DIR}/tests/stubs" -I "${ROOT_DIR}" \
  "${ROOT_DIR}/tests/test_directional_leds_controller.cpp" \
  "${ROOT_DIR}/esphome/components/satellite1_directional_leds/directional_leds.cpp" \
  -o "${TEST_DIR}/directional_leds_controller"
"${TEST_DIR}/directional_leds_controller"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I "${ROOT_DIR}/tests/stubs" -I "${ROOT_DIR}" \
  "${ROOT_DIR}/tests/test_audio_hints.cpp" \
  "${ROOT_DIR}/esphome/components/satellite1_audio_hints/audio_hints.cpp" \
  "${ROOT_DIR}/esphome/components/satellite1_directional_leds/directional_leds.cpp" \
  -o "${TEST_DIR}/audio_hints"
"${TEST_DIR}/audio_hints"
