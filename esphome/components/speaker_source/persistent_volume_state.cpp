#include "persistent_volume_state.h"

namespace esphome::speaker_source {

void PersistentVolumeState::initialize() {
  if (this->is_initialized()) {
    return;
  }
  VolumeRestoreState restored{};
  if (!this->preference_.load(&restored) || !std::isfinite(restored.volume) || restored.volume < 0.0f ||
      restored.volume > 1.0f) {
    // The original first-boot default is unmuted, even when initial volume is zero.
    restored = {this->initial_volume_, false};
  }
  if (!std::isfinite(restored.volume) || restored.volume < 0.0f || restored.volume > 1.0f) {
    return;
  }
  this->preference_.save(&restored);
  this->publish_state_(restored.volume, restored.is_muted);
}

bool PersistentVolumeState::set_volume(float volume) {
  if (!this->is_initialized() || !std::isfinite(volume) || volume < 0.0f || volume > 1.0f) {
    return false;
  }
  const auto current = this->state();
  if (current.volume != volume) {
    const VolumeRestoreState restored{volume, current.muted};
    this->preference_.save(&restored);
    this->publish_state_(volume, current.muted);
  }
  return true;
}

bool PersistentVolumeState::set_muted(bool muted) {
  if (!this->is_initialized()) {
    return false;
  }
  const auto current = this->state();
  if (current.muted != muted) {
    const VolumeRestoreState restored{current.volume, muted};
    this->preference_.save(&restored);
    this->publish_state_(current.volume, muted);
  }
  return true;
}

}  // namespace esphome::speaker_source
