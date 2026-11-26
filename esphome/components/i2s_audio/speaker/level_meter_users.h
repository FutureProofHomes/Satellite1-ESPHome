#pragma once
#include <atomic>
#include <cstdint>

namespace esphome::i2s_audio {
enum class PlaybackLevelUser : uint8_t { DIRECTIONAL_LEDS = 1, AUDIO_HINTS = 2 };

// Each consumer releases only its own read-only metering request.
class LevelMeterUsers {
 public:
  void set_enabled(PlaybackLevelUser user, bool enabled) {
    const auto mask = static_cast<uint8_t>(user);
    if (enabled)
      this->users_.fetch_or(mask, std::memory_order_relaxed);
    else
      this->users_.fetch_and(static_cast<uint8_t>(~mask), std::memory_order_relaxed);
  }
  bool enabled() const { return this->users_.load(std::memory_order_relaxed) != 0; }

 protected:
  std::atomic<uint8_t> users_{0};
};
}  // namespace esphome::i2s_audio
