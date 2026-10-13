#include "volume_state.h"

#include <cmath>
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *const TAG = "satellite1.volume_state";
static bool valid_volume(float volume) { return std::isfinite(volume) && volume >= 0.0f && volume <= 1.0f; }

void DACOutputVolumeState::initialize() {
  if (this->initialization_started_ || this->parent_ == nullptr)
    return;
  this->initialization_started_ = true;
  this->parent_->set_volume_state(this);
  this->pref_ =
      global_preferences->make_preference<DACOutputVolumePreferences>(fnv1_hash("satellite1.volume_state.v1"));
  DACOutputVolumePreferences restored{};
  if (this->pref_.load(&restored) && restored.version == 1 && valid_volume(restored.speaker_volume) &&
      valid_volume(restored.line_out_volume)) {
    this->restore_state_ = restored;
  } else {
    auto legacy_pref = this->parent_->make_entity_preference<DACProxyRestoreState>();
    DACProxyRestoreState legacy{};
    if (legacy_pref.load(&legacy) && (legacy.dac_output == SPEAKER || legacy.dac_output == LINE_OUT) &&
        valid_volume(legacy.speaker_volume) && valid_volume(legacy.line_out_volume)) {
      this->restore_state_.speaker_volume = clamp((legacy.speaker_volume - 0.1f) / 0.9f, 0.0f, 1.0f);
      this->restore_state_.line_out_volume = clamp((legacy.line_out_volume - 0.1f) / 0.9f, 0.0f, 1.0f);
      this->restore_state_.global_muted =
          legacy.dac_output == SPEAKER ? legacy.speaker_is_muted : legacy.line_out_is_muted;
      ESP_LOGD(TAG, "Migrated legacy logical volume preferences");
    }
    this->save_();
  }
  if (this->parent_->has_requested_output_notification())
    this->publish_selected_state_(true);
}

void DACOutputVolumeState::publish_selected_state_(bool force) {
  if (!this->parent_->has_requested_output_notification())
    return;
  const bool route_changed = this->is_initialized() && this->selected_output_ != this->parent_->requested_output();
  this->selected_output_ = this->parent_->requested_output();
  this->publish_state_(
      this->selected_output_ == SPEAKER ? this->restore_state_.speaker_volume : this->restore_state_.line_out_volume,
      this->restore_state_.global_muted, force || route_changed);
}

void DACOutputVolumeState::save_() {
  if (!this->pref_.save(&this->restore_state_))
    ESP_LOGW(TAG, "Failed to save logical volume preferences");
}

bool DACOutputVolumeState::set_volume(float volume) {
  if (!this->is_initialized() || !valid_volume(volume))
    return false;
  this->selected_output_ = this->parent_->requested_output();
  float &stored =
      this->selected_output_ == SPEAKER ? this->restore_state_.speaker_volume : this->restore_state_.line_out_volume;
  if (stored != volume) {
    stored = volume;
    this->save_();
    this->publish_state_(volume, this->restore_state_.global_muted);
  }
  return true;
}

bool DACOutputVolumeState::set_muted(bool muted) {
  if (!this->is_initialized())
    return false;
  this->selected_output_ = this->parent_->requested_output();
  if (this->restore_state_.global_muted != muted) {
    this->restore_state_.global_muted = muted;
    this->save_();
    this->publish_state_(
        this->selected_output_ == SPEAKER ? this->restore_state_.speaker_volume : this->restore_state_.line_out_volume,
        muted);
  }
  return true;
}

}  // namespace satellite1
}  // namespace esphome
