#include "dac_proxy.h"

#include <algorithm>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *const TAG = "dac_proxy";

void DACProxy::setup() {
  ESP_LOGD(TAG, "Setting up DACProxy...");
  this->pref_ = this->make_entity_preference<DACProxyRestoreState>();

  if (this->pref_.load(&this->restore_state_)) {
    ESP_LOGD(TAG, "Read preferences from flash");
    ESP_LOGD(TAG, "   active dac: %d", this->restore_state_.dac_output);
    this->active_dac = (DacOutput) this->restore_state_.dac_output;
    // The active output's slot is authoritative, and both DACs come up at that one level: there is
    // one level behind both outputs, so an output that came back at a level of its own would be
    // wrong the moment the jack is plugged in.
    this->level_ =
        (this->active_dac == LINE_OUT) ? this->restore_state_.line_out_volume : this->restore_state_.speaker_volume;
  } else {
    ESP_LOGW(TAG, "Preferences not found, using default settings");
    this->active_dac = LINE_OUT;
    this->level_ = .5;
  }
  this->level_ = clamp<float>(this->level_, 0.0f, 1.0f);
  // Mute first, so the TAS2780 records the level without writing it over its mute code.
  // XMOS owns the I2S clocks. Keep both paths muted until audio routing is released.
  if (this->pcm5122_) {
    this->pcm5122_->set_mute_on();
    this->pcm5122_->set_volume(this->level_);
  }
  if (this->tas2780_) {
    this->tas2780_->set_mute_on();
    this->tas2780_->set_volume(this->level_);
  }
  this->setup_was_called_ = true;
  this->defer([this]() { this->state_callback_.call(); });
}

void DACProxy::dump_config() {
  if (this->tas2780_) {
    esph_log_config(TAG, "SPEAKER-DAC, volume: %4.2f, muted: %s %s", this->tas2780_->volume(),
                    this->tas2780_->is_muted() ? "true" : "false", this->active_dac == SPEAKER ? "(active)" : "");
  }
  if (this->pcm5122_) {
    esph_log_config(TAG, "LINE-OUT-DAC, volume: %4.2f, muted: %s %s", this->pcm5122_->volume(),
                    this->pcm5122_->is_muted() ? "true" : "false", this->active_dac == LINE_OUT ? "(active)" : "");
  }
}

void DACProxy::save_restore_state_() {
  this->restore_state_.dac_output = this->active_dac;
  // Both slots get the level, because there is only one level to remember - see setup(). The mute
  // flags are kept for the layout only: units saved at volume 0 by older firmware have them set,
  // and gating an unmute on them would leave those units silent for good.
  this->restore_state_.speaker_volume = this->level_;
  this->restore_state_.speaker_is_muted = false;
  this->restore_state_.line_out_volume = this->level_;
  this->restore_state_.line_out_is_muted = false;
  this->pref_.save(&this->restore_state_);
}

void DACProxy::activate_line_out() {
  if (this->pcm5122_ == nullptr) {
    return;
  }
  ESP_LOGD(TAG, "Activate Line-Out DAC.");
  this->active_dac = LINE_OUT;

  if (this->tas2780_) {
    this->tas2780_->set_mute_on();
  }
  this->send_selected_dac_();
  // The DAC just switched to may have missed level changes while it was inactive, so it is set
  // before it is unmuted.
  this->apply_level_();
  this->pcm5122_->set_mute_off();
  this->defer([this]() { this->state_callback_.call(); });
  this->save_restore_state_();
}

void DACProxy::activate_speaker() {
  if (this->tas2780_ == nullptr) {
    return;
  }
  ESP_LOGD(TAG, "Activate Speaker DAC.");
  this->active_dac = SPEAKER;
  this->send_selected_dac_();
  if (this->pcm5122_) {
    this->pcm5122_->set_mute_on();
  }
  // See activate_line_out().
  this->apply_level_();
  this->tas2780_->set_mute_off();
  this->defer([this]() { this->state_callback_.call(); });
  this->save_restore_state_();
}

void DACProxy::activate() {
  if (this->active_dac == SPEAKER && this->tas2780_) {
    if (this->pcm5122_) {
      this->pcm5122_->set_mute_on();
    }
    this->apply_level_();
    this->tas2780_->set_mute_off();
  } else if (this->pcm5122_) {
    if (this->tas2780_) {
      this->tas2780_->set_mute_on();
    }
    this->apply_level_();
    this->pcm5122_->set_mute_off();
  }
}

bool DACProxy::set_level(float level) {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_level() called before setup()");
    return false;
  }
  level = clamp<float>(level, 0.0f, 1.0f);
  if (level == this->level_) {
    return this->apply_level_();
  }
  this->level_ = level;
  const bool ret = this->apply_level_();
  this->save_restore_state_();
  return ret;
}

bool DACProxy::apply_level_() {
  audio_dac::AudioDac *dac = (this->active_dac == LINE_OUT) ? this->pcm5122_ : this->tas2780_;
  if (dac == nullptr) {
    return false;
  }
  // The reconciler restates its level on every run, so most calls here ask for a level the DAC is
  // already at; returning early keeps those off the I2C bus.
  if (dac->volume() == this->level_) {
    return false;
  }
  return dac->set_volume(this->level_);
}

bool DACProxy::is_muted() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::is_muted() called before setup()");
    return false;
  }
  if (this->active_dac == LINE_OUT && this->pcm5122_) {
    return this->pcm5122_->is_muted();
  }
  if (this->active_dac == SPEAKER && this->tas2780_) {
    return this->tas2780_->is_muted();
  }
  return false;
}

float DACProxy::volume() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::volume() called before setup()");
    return .5;
  }
  if (this->active_dac == LINE_OUT && this->pcm5122_) {
    return this->pcm5122_->volume();
  }
  if (this->active_dac == SPEAKER && this->tas2780_) {
    return this->tas2780_->volume();
  }
  return 0.;
}

}  // namespace satellite1
}  // namespace esphome
