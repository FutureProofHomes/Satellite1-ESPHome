#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace esphome::satellite1 {

enum class HardwareMute : uint8_t { UNKNOWN, OFF, ON };
// Five missed 20 ms controller reports invalidate microphone admission.
static constexpr uint32_t MICROPHONE_GPIO_MAX_AGE_MS = 100;

// Single main-loop writer; microphone workers only read. Unknown hardware is
// inhibition, not evidence that the physical switch is ON.
class MicrophoneMuteState {
 public:
  void observe_hardware(bool muted, uint32_t now) {
    this->sequence_.fetch_add(1);
    this->observed_at_.store(now);
    this->hardware_.store(muted ? HardwareMute::ON : HardwareMute::OFF);
    this->sequence_.fetch_add(1);
  }
  void invalidate_hardware() {
    this->sequence_.fetch_add(1);
    this->hardware_.store(HardwareMute::UNKNOWN);
    this->sequence_.fetch_add(1);
  }
  HardwareMute hardware_state(uint32_t now) const {
    const auto sequence = this->sequence_.load();
    const auto hardware = this->hardware_.load();
    const auto observed_at = this->observed_at_.load();
    if ((sequence & 1) || sequence != this->sequence_.load() ||
        static_cast<uint32_t>(now - observed_at) >= MICROPHONE_GPIO_MAX_AGE_MS)
      return HardwareMute::UNKNOWN;
    return hardware;
  }
  void restore_software(bool muted) { this->software_.store(muted); }
  bool request_software(bool muted, uint32_t now) {
    if (!muted && this->hardware_state(now) == HardwareMute::ON)
      return false;
    this->software_.store(muted);
    return true;
  }
  bool software_muted() const { return this->software_.load(); }
  bool effectively_muted(uint32_t now) const {
    return this->software_muted() || this->hardware_state(now) != HardwareMute::OFF;
  }
  bool local_capture_allowed(uint32_t now) const { return this->hardware_state(now) == HardwareMute::OFF; }
  void expire_hardware(uint32_t now) {
    // Retire expired observations so a full millis() wrap cannot revive them.
    if (this->hardware_.load() != HardwareMute::UNKNOWN && this->hardware_state(now) == HardwareMute::UNKNOWN)
      this->invalidate_hardware();
  }

  // Interleaved 32-bit microphone frames. Software mute may preserve selected
  // local-only channels; hardware inhibition always silences every channel.
  void filter_samples(int32_t *samples, size_t count, uint8_t channels, uint8_t software_mute_channels,
                      uint32_t now) const {
    const bool hardware_inhibited = !this->local_capture_allowed(now);
    const bool software_muted = this->software_muted();
    for (size_t i = 0; i < count; ++i) {
      if (hardware_inhibited || (software_muted && (software_mute_channels & (1u << (i % channels)))))
        samples[i] = 0;
    }
  }

 protected:
  std::atomic<uint32_t> sequence_{0};
  std::atomic<uint32_t> observed_at_{0};
  std::atomic<HardwareMute> hardware_{HardwareMute::UNKNOWN};
  // Fail closed until the software preference owner has restored its value.
  std::atomic<bool> software_{true};
};

}  // namespace esphome::satellite1
