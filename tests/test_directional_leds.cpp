#include "esphome/components/satellite1_directional_leds/direction_state.h"
#include "esphome/components/i2s_audio/speaker/pcm_level.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using esphome::satellite1_directional_leds::DirectionState;
using esphome::i2s_audio::pcm_level_q16;

static std::array<uint8_t, 32> frame(uint32_t counter, uint8_t angle, uint8_t confidence = 40, uint8_t flags = 3) {
  std::array<uint8_t, 32> data{};
  data[2] = confidence;
  data[3] = flags;
  for (int i = 0; i < 4; ++i)
    data[8 + i] = counter >> (i * 8);
  data[14] = angle;
  return data;
}

static void observe(DirectionState &state, uint32_t counter, uint8_t angle, uint32_t now, bool listening = true,
                    uint8_t confidence = 40, uint8_t flags = 3) {
  auto data = frame(counter, angle, confidence, flags);
  state.observe(data.data(), now, listening);
}

static void direction_tests() {
  DirectionState state;
  observe(state, 1, 23, 0);
  assert(!state.progressing(0));  // A version/one snapshot alone does not prove live DSP.
  observe(state, 2, 23, 50);
  assert(state.has_direction(50));
  observe(state, 3, 1, 100);
  assert(state.position() > 23 || state.position() < 1);  // Cross the wrap, not the back of the ring.
  assert(state.progressing(600));
  assert(!state.progressing(601));
  observe(state, 3, 12, 700);
  assert(!state.progressing(700));  // A frozen counter cannot refresh the direction.

  state.reset();
  observe(state, 20, 6, 1000);
  for (uint32_t i = 1; i <= 30; ++i)
    observe(state, 20 + i, 6, 1000 + i * 50);
  observe(state, 51, 18, 2550);  // A short later sound must not replace the established speaker for the reply.
  assert(state.dominant() == 6);
  assert(state.has_remembered_direction());
  const auto remembered = state.dominant();
  observe(state, 52, 18, 2600, false);  // Playback is not allowed to learn a new speaker position.
  assert(state.dominant() == remembered);

  state.begin_listening();
  assert(state.progressing(2600));
  assert(!state.has_remembered_direction() && !state.has_direction(2600));
  observe(state, 53, 18, 2650);
  assert(state.has_direction(2650) && state.dominant() == 18);  // Keep liveness, discard old speaker votes.

  state.reset();
  observe(state, 1, 6, 0);
  observe(state, 2, 6, 50, true, 11);  // Weak estimates and invalid flags stay neutral.
  observe(state, 3, 6, 100, true, 40, 2);
  observe(state, 4, 24, 150);
  assert(state.progressing(150));
  assert(!state.has_direction(150));
  assert(!state.has_remembered_direction());
  observe(state, 5, 6, 200);
  for (uint32_t i = 6; i < 24; ++i)
    observe(state, i, 18, 200 + (i - 5) * 50, true, 0, 2);
  assert(!state.has_direction(1100));  // Keep frame progress separate from the bounded direction hold.
  assert(state.has_remembered_direction());
  state.reset();
  assert(!state.has_remembered_direction());  // No previous conversation leaks into a new session.

  observe(state, UINT32_MAX, 9, UINT32_MAX - 30);
  observe(state, 0, 9, 20);
  assert(state.has_direction(30));  // Frame and millis wrap are both legal.
}

static std::vector<uint8_t> pcm(uint8_t width, int64_t left, int64_t right) {
  std::vector<uint8_t> data;
  for (int frame = 0; frame < 32; ++frame) {
    for (auto value : {left, right}) {
      const uint32_t raw = static_cast<uint32_t>(value);
      for (int byte = 0; byte < width; ++byte)
        data.push_back(raw >> (byte * 8));
    }
  }
  return data;
}

static void level_tests() {
  for (uint8_t width = 1; width <= 4; ++width) {
    const int64_t full_scale = int64_t(1) << (width * 8 - 1);
    auto data = pcm(width, 0, -full_scale);
    const auto original = data;
    const auto level = pcm_level_q16(data.data(), data.size(), width, 2);
    assert(level >= 32767 && level <= 32768);  // Both stereo channels, including the quiet left channel.
    assert(data == original);
    data = pcm(width, -full_scale, -full_scale);
    assert(pcm_level_q16(data.data(), data.size(), width, 2) == 65535);  // INT32_MIN cannot overflow.
    data = pcm(width, 0, 0);
    assert(pcm_level_q16(data.data(), data.size(), width, 2) == 0);
    assert(pcm_level_q16(data.data(), width - 1, width, 2) == 0);  // Ignore incomplete sample/frame tails.
  }
  auto data = pcm(4, 0, 0);
  assert(pcm_level_q16(data.data(), data.size(), 0, 2) == 0);
  assert(pcm_level_q16(data.data(), data.size(), 4, 0) == 0);
}

int main() {
  direction_tests();
  level_tests();
  std::cout << "Direction tracking, reply memory, freshness and PCM level checks passed.\n";
}
