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
    // a single media player volume behind all of this, so an output that came back at a level of
    // its own would disagree with the slider the moment the jack is plugged in.
    //
    // The floor stays at 0 until the reconciler runs on boot, so until then the DAC sits at exactly
    // the restored level and nothing here depends on the reconciler having an answer.
    this->requested_volume_ =
        (this->active_dac == LINE_OUT) ? this->restore_state_.line_out_volume : this->restore_state_.speaker_volume;
    if (this->pcm5122_) {
      this->pcm5122_->set_volume(this->requested_volume_);
      if (this->restore_state_.line_out_is_muted) {
        this->pcm5122_->set_mute_on();
      }
    }
    if (this->tas2780_) {
      this->tas2780_->set_volume(this->requested_volume_);
      if (this->restore_state_.speaker_is_muted) {
        this->tas2780_->set_mute_on();
      }
    }
    this->activate();
  } else {
    ESP_LOGW(TAG, "Preferences not found, using default settings");
    this->active_dac = LINE_OUT;
    this->restore_state_.dac_output = LINE_OUT;
    this->restore_state_.speaker_volume = .5;
    this->restore_state_.speaker_is_muted = false;
    this->restore_state_.line_out_volume = .5;
    this->restore_state_.line_out_is_muted = false;
    this->requested_volume_ = .5;
    if (this->pcm5122_) {
      this->pcm5122_->set_volume(this->requested_volume_);
    }
    if (this->tas2780_) {
      this->tas2780_->set_volume(this->requested_volume_);
    }
  }
  this->setup_was_called_ = true;
  this->defer([this]() { this->state_callback_.call(); });
}

void DACProxy::dump_config() {
  if (this->tas2780_) {
    esph_log_config(TAG, "SPEAKER-DAC, volume: %4.2f, muted: %s %s", this->tas2780_->volume(),
                    this->restore_state_.speaker_is_muted ? "true" : "false",
                    this->active_dac == SPEAKER ? "(active)" : "");
  }
  if (this->pcm5122_) {
    esph_log_config(TAG, "LINE-OUT-DAC, volume: %4.2f, muted: %s %s", this->pcm5122_->volume(),
                    this->restore_state_.line_out_is_muted ? "true" : "false",
                    this->active_dac == LINE_OUT ? "(active)" : "");
  }
}

void DACProxy::save_volume_restore_state_() {
  ESP_LOGD(TAG, "Saving volume restore state...");
  ESP_LOGD(TAG, "Active DAC: %d", this->active_dac);

  this->restore_state_.dac_output = this->active_dac;
  // requested_volume_ rather than the level the DAC is at: the reconciler raises the DAC for voice,
  // and only the ducking applied at the same time keeps that safe. Persisting the raised level would
  // restore it on the next boot with no ducking behind it. Both slots get it, because there is only
  // one volume to remember - see setup().
  this->restore_state_.speaker_volume = this->requested_volume_;
  this->restore_state_.line_out_volume = this->requested_volume_;
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
  if (!this->restore_state_.line_out_is_muted) {
    this->pcm5122_->set_mute_off();
  }
  this->send_selected_dac_();
  // Only the DAC that was live tracks the gain floor, so the one just switched to is still at the
  // bare requested volume. Re-applying here puts a jack plugged in mid-response at the right level
  // immediately, rather than waiting out audio_gain_raise's delay.
  this->apply_volume_();
  this->defer([this]() { this->state_callback_.call(); });
  this->save_volume_restore_state_();
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
  if (!this->restore_state_.speaker_is_muted) {
    this->tas2780_->set_mute_off();
  }
  // See activate_line_out().
  this->apply_volume_();
  this->defer([this]() { this->state_callback_.call(); });
  this->save_volume_restore_state_();
}

void DACProxy::activate() {
  if (this->active_dac == SPEAKER && this->tas2780_) {
    if (this->pcm5122_) {
      this->pcm5122_->set_mute_on();
    }
    if (!this->restore_state_.speaker_is_muted) {
      this->tas2780_->set_mute_off();
    }
  } else if (this->pcm5122_) {
    if (this->tas2780_) {
      this->tas2780_->set_mute_on();
    }
    if (!this->restore_state_.line_out_is_muted) {
      this->pcm5122_->set_mute_off();
    }
  }
  // Same invariant the other two activation paths hold: after activating, the live DAC is at
  // max(requested_volume_, volume_floor_). A no-op on the boot path, where the floor is still 0.
  this->apply_volume_();
}

bool DACProxy::set_mute_off() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_mute_off() called before setup()");
    return false;
  }
  bool has_changed = false;
  bool ret = false;
  if (this->active_dac == LINE_OUT && this->pcm5122_ && this->pcm5122_->is_muted()) {
    ret = this->pcm5122_->set_mute_off();
    this->restore_state_.line_out_is_muted = false;
    has_changed = true;
  }
  if (this->active_dac == SPEAKER && this->tas2780_ && this->tas2780_->is_muted()) {
    ret = this->tas2780_->set_mute_off();
    this->restore_state_.speaker_is_muted = false;
    has_changed = true;
  }
  if (has_changed) {
    ESP_LOGD(TAG, "set_mute_off: for %s", this->active_dac == LINE_OUT ? "Line-Out" : "Speaker");
    this->save_volume_restore_state_();
  }
  return ret;
}

bool DACProxy::set_mute_on() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_mute_on() called before setup()");
    return false;
  }
  bool has_changed = false;
  bool ret = false;
  if (this->active_dac == LINE_OUT && this->pcm5122_ && !this->pcm5122_->is_muted()) {
    ret = this->pcm5122_->set_mute_on();
    this->restore_state_.line_out_is_muted = true;
    has_changed = true;
  }
  if (this->active_dac == SPEAKER && this->tas2780_ && !this->tas2780_->is_muted()) {
    ret = this->tas2780_->set_mute_on();
    this->restore_state_.speaker_is_muted = true;
    has_changed = true;
  }
  if (has_changed) {
    ESP_LOGD(TAG, "set_mute_on: for %s", this->active_dac == LINE_OUT ? "Line-Out" : "Speaker");
    this->save_volume_restore_state_();
  }
  return ret;
}

bool DACProxy::set_volume(float volume) {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_volume() called before setup()");
    return false;
  }
  this->requested_volume_ = clamp<float>(volume, 0.0f, 1.0f);
  const bool ret = this->apply_volume_();
  // Saved whether or not apply_volume_() reached the hardware: under an active floor the DAC already
  // sits above requested_volume_ and the write is skipped, so without the save here a media volume
  // changed during a voice response would be forgotten by the next boot.
  this->save_volume_restore_state_();
  return ret;
}

bool DACProxy::set_volume_floor(float floor) {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_volume_floor() called before setup()");
    return false;
  }
  this->volume_floor_ = clamp<float>(floor, 0.0f, 1.0f);
  return this->apply_volume_();
}

bool DACProxy::apply_volume_() {
  const float applied = std::max(this->requested_volume_, this->volume_floor_);

  audio_dac::AudioDac *dac = (this->active_dac == LINE_OUT) ? this->pcm5122_ : this->tas2780_;
  if (dac == nullptr) {
    return false;
  }
  // Both writers restate their own value on every reconcile, so most calls here ask for a level the
  // DAC is already at; returning early keeps those off the I2C bus. Persisting is not this
  // function's job: only set_volume() changes what is worth remembering.
  if (dac->volume() == applied) {
    return false;
  }
  return dac->set_volume(applied);
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