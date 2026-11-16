#pragma once

#include "esphome/core/preferences.h"
#include "volume_state.h"

namespace esphome::speaker_source {

// Keep the shipped player preference layout, including its original mute field.
struct VolumeRestoreState {
  float volume;
  bool is_muted;
};

class PersistentVolumeState : public VolumeState {
 public:
  void set_preference(ESPPreferenceObject preference) { this->preference_ = preference; }
  void set_initial_volume(float volume) { this->initial_volume_ = volume; }
  void initialize() override;
  bool set_volume(float volume) override;
  bool set_muted(bool muted) override;

 protected:
  ESPPreferenceObject preference_;
  float initial_volume_{0.5f};
};

}  // namespace esphome::speaker_source
