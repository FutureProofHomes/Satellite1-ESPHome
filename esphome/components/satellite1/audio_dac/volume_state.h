#pragma once

#include "dac_proxy.h"
#include "esphome/components/speaker_source/volume_state.h"

namespace esphome {
namespace satellite1 {

struct DACOutputVolumePreferences {
  uint8_t version{1};
  float speaker_volume{0.5f};
  float line_out_volume{0.5f};
  bool global_muted{false};
};

class DACOutputVolumeState : public speaker_source::VolumeState {
  friend class DACProxy;

 public:
  void set_parent(DACProxy *parent) { this->parent_ = parent; }
  void initialize() override;
  bool set_volume(float volume) override;
  bool set_muted(bool muted) override;

 protected:
  void publish_selected_state_(bool force = false);
  void save_();
  DACProxy *parent_{nullptr};
  ESPPreferenceObject pref_;
  DACOutputVolumePreferences restore_state_{};
  DacOutput selected_output_{LINE_OUT};
  bool initialization_started_{false};
};

}  // namespace satellite1
}  // namespace esphome
