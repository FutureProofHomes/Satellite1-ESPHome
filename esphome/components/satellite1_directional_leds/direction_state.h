#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace esphome::satellite1_directional_leds {

// Direction filtering and remembered speaker position, independent of the LED renderer.
// Adapted from Tater Native's Sat1 listening and reply direction handling.
class DirectionState {
 public:
  static constexpr uint8_t LED_COUNT = 24;
  static constexpr uint32_t PROGRESS_TIMEOUT_MS = 500;
  void reset() { *this = DirectionState{}; }

  // A new conversation discards speaker evidence, not the known XMOS frame
  // counter. Clearing transport liveness here would force an effect change.
  void begin_listening() {
    this->votes_.fill(0);
    this->direction_at_ = 0;
    this->position_ = LED_COUNT / 2.0f;
    this->dominant_ = LED_COUNT / 2;
    this->have_direction_ = false;
  }

  void observe(const uint8_t *payload, uint32_t now, bool listening) {
    const uint32_t frame = uint32_t(payload[8]) | (uint32_t(payload[9]) << 8) | (uint32_t(payload[10]) << 16) |
                           (uint32_t(payload[11]) << 24);
    const bool advanced = this->have_frame_ && frame != this->frame_;
    this->have_frame_ = true;
    this->frame_ = frame;
    if (!advanced)
      return;
    this->have_progress_ = true;
    this->progress_at_ = now;
    // VALID + FOUR_MIC, sufficient confidence, and an in-range physical ring position.
    if (!listening || (payload[3] & 3) != 3 || payload[2] < 12 || payload[14] >= LED_COUNT)
      return;

    const float target = payload[14];
    if (!this->has_direction(now)) {
      this->position_ = target;
    } else {
      float difference = target - this->position_;
      if (difference > LED_COUNT / 2.0f)
        difference -= LED_COUNT;
      if (difference < -LED_COUNT / 2.0f)
        difference += LED_COUNT;
      this->position_ = wrap(this->position_ + difference * 0.42f);
    }
    this->have_direction_ = true;
    this->direction_at_ = now;

    // Accumulate listening evidence so a brief late noise does not redirect the reply.
    if (this->votes_[this->dominant_] > std::numeric_limits<uint32_t>::max() - 1024) {
      for (auto &vote : this->votes_)
        vote >>= 1;
    }
    const uint8_t index = payload[14];
    const uint32_t weight = 256 + std::min<uint8_t>(payload[2], 64);
    this->votes_[index] += weight;
    this->votes_[(index + 1) % LED_COUNT] += weight / 4;
    this->votes_[(index + LED_COUNT - 1) % LED_COUNT] += weight / 4;
    for (uint8_t i = 0; i < LED_COUNT; ++i) {
      if (this->votes_[i] > this->votes_[this->dominant_])
        this->dominant_ = i;
    }
  }

  bool progressing(uint32_t now) const {
    return this->have_progress_ && now - this->progress_at_ <= PROGRESS_TIMEOUT_MS;
  }
  bool has_direction(uint32_t now) const {
    return this->have_direction_ && this->progressing(now) && now - this->direction_at_ <= 800;
  }
  bool has_remembered_direction() const { return this->have_direction_; }
  float position() const { return this->position_; }
  uint8_t dominant() const { return this->dominant_; }

 protected:
  static float wrap(float value) {
    if (value < 0)
      value += LED_COUNT;
    if (value >= LED_COUNT)
      value -= LED_COUNT;
    return value;
  }
  std::array<uint32_t, LED_COUNT> votes_{};
  uint32_t frame_{0};
  uint32_t progress_at_{0};
  uint32_t direction_at_{0};
  float position_{LED_COUNT / 2.0f};
  uint8_t dominant_{LED_COUNT / 2};
  bool have_frame_{false};
  bool have_progress_{false};
  bool have_direction_{false};
};

}  // namespace esphome::satellite1_directional_leds
