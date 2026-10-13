#include "dac_proxy.h"
#include "volume_state.h"

#include <cmath>
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *const TAG = "dac_proxy";
static constexpr uint8_t DAC_WRITE_ATTEMPTS = 3;

static bool valid_volume(float volume) { return std::isfinite(volume) && volume >= 0.0f && volume <= 1.0f; }
static bool valid_output(uint8_t output) { return output == SPEAKER || output == LINE_OUT; }
static bool valid_volume_limits(DacOutput output, float min_db, float max_db) {
  return valid_output(output) && std::isfinite(min_db) && std::isfinite(max_db) && min_db < max_db &&
         min_db >= (output == SPEAKER ? -100.0f : -103.0f) && max_db <= (output == SPEAKER ? 0.0f : 24.0f);
}

void DACProxy::setup() {
  this->volume_limits_pref_ = global_preferences->make_preference<DACProxyVolumeLimitPreferences>(
      fnv1_hash("satellite1.dac_proxy.volume_limits.v1"));
  DACProxyVolumeLimitPreferences limits{};
  if (this->volume_limits_pref_.load(&limits) && limits.version == 1 &&
      valid_volume_limits(SPEAKER, limits.speaker.min_db, limits.speaker.max_db) &&
      valid_volume_limits(LINE_OUT, limits.line_out.min_db, limits.line_out.max_db)) {
    this->volume_limits_ = limits;
  } else if (!this->volume_limits_pref_.save(&this->volume_limits_)) {
    ESP_LOGW(TAG, "Failed to save default DAC volume limits");
  }
  this->apply_volume_limits_(SPEAKER);
  this->apply_volume_limits_(LINE_OUT);
  this->pref_ =
      global_preferences->make_preference<DACProxyRoutingPreferences>(fnv1_hash("satellite1.dac_proxy.routing.v1"));
  DACProxyRoutingPreferences restored{};
  if (this->pref_.load(&restored) && restored.version == 1 && valid_output(restored.dac_output)) {
    this->restore_state_ = restored;
  } else {
    auto legacy_pref = this->make_entity_preference<DACProxyRestoreState>();
    DACProxyRestoreState legacy{};
    if (legacy_pref.load(&legacy) && valid_output(legacy.dac_output) && valid_volume(legacy.speaker_volume) &&
        valid_volume(legacy.line_out_volume)) {
      this->restore_state_.dac_output = legacy.dac_output;
      ESP_LOGD(TAG, "Migrated legacy DAC routing");
    }
    if (!this->pref_.save(&this->restore_state_))
      ESP_LOGW(TAG, "Failed to save DAC routing preferences");
  }
  this->active_dac = static_cast<DacOutput>(this->restore_state_.dac_output);
  this->requested_dac_ = this->active_dac;
  this->setup_was_called_ = true;
  this->switching_output_ = true;
  this->reconcile_();
  this->defer([this]() {
    this->requested_output_notified_ = true;
    if (this->volume_state_ != nullptr)
      this->volume_state_->publish_selected_state_(true);
    this->requested_output_callback_.call();
    this->output_change_callback_.call();
    this->selection_callback_.call();
    this->state_callback_.call();
    this->switching_output_ = false;
    this->reconcile_();
  });
}

void DACProxy::dump_config() {
  ESP_LOGCONFIG(TAG, "DAC Proxy: selected=%s, target volume=%.2f, target muted=%s, audio ready=%s",
                this->active_dac == SPEAKER ? "speaker" : "line-out", this->target_volume_, YESNO(this->target_muted_),
                YESNO(this->audio_ready()));
  ESP_LOGCONFIG(TAG, "  Speaker volume limits: %.1f to %.1f dB", this->volume_limits_.speaker.min_db,
                this->volume_limits_.speaker.max_db);
  ESP_LOGCONFIG(TAG, "  Line-out volume limits: %.1f to %.1f dB", this->volume_limits_.line_out.min_db,
                this->volume_limits_.line_out.max_db);
}

void DACProxy::apply_volume_limits_(DacOutput output) {
  if (this->volume_limits_callbacks_[output]) {
    const auto &limits = output == SPEAKER ? this->volume_limits_.speaker : this->volume_limits_.line_out;
    this->volume_limits_callbacks_[output](limits.min_db, limits.max_db);
  }
}

bool DACProxy::set_volume_limits(DacOutput output, float min_db, float max_db) {
  if (!this->setup_was_called_ || !valid_volume_limits(output, min_db, max_db)) {
    ESP_LOGW(TAG, "Rejected invalid or premature DAC volume-limit request");
    return false;
  }
  if (!(output == SPEAKER ? this->tas2780_ : this->pcm5122_) || !this->volume_limits_callbacks_[output]) {
    ESP_LOGW(TAG, "Rejected volume limits for an unavailable DAC");
    return false;
  }
  auto updated = this->volume_limits_;
  auto &limits = output == SPEAKER ? updated.speaker : updated.line_out;
  if (limits.min_db != min_db || limits.max_db != max_db) {
    limits = {min_db, max_db};
    if (!this->volume_limits_pref_.save(&updated)) {
      ESP_LOGW(TAG, "Failed to save DAC volume limits; limits unchanged");
      return false;
    }
    this->volume_limits_ = updated;
  }
  this->apply_volume_limits_(output);
  (output == SPEAKER ? this->speaker_applied_ : this->lineout_applied_).volume_valid = false;
  // A non-selected or suspended output receives its gain when normal routing permits it.
  return output != this->active_dac || !this->audio_ready() || this->reconcile_();
}

bool DACProxy::apply_mute_(audio_dac::AudioDac *dac, AppliedState &state, bool muted) {
  if (state.mute_valid && state.muted == muted && dac->is_muted() == muted)
    return true;
  state.mute_valid = false;
  for (uint8_t attempt = 0; attempt < DAC_WRITE_ATTEMPTS; ++attempt) {
    if (muted ? dac->set_mute_on() : dac->set_mute_off()) {
      state.muted = muted;
      state.mute_valid = true;
      return true;
    }
    // Mute may share volume registers; a failed write confirms neither value.
    state.volume_valid = false;
  }
  ESP_LOGW(TAG, "DAC %s failed after %u attempts", muted ? "mute" : "unmute", DAC_WRITE_ATTEMPTS);
  return false;
}

bool DACProxy::reconcile_() {
  bool success = true;
  auto *selected =
      valid_output(this->active_dac) ? (this->active_dac == SPEAKER ? this->tas2780_ : this->pcm5122_) : nullptr;
  const bool muted = !this->audio_ready() || selected == nullptr || this->target_muted_;
  // Driver caches can change before a failed write, so only successful calls confirm applied state.
  for (auto *dac : {this->tas2780_, this->pcm5122_}) {
    if (dac == nullptr)
      continue;
    auto &state = dac == this->tas2780_ ? this->speaker_applied_ : this->lineout_applied_;
    if (state.volume_valid && dac->volume() != state.volume)
      state.volume_valid = false;
    if (state.mute_valid && dac->is_muted() != state.muted)
      state.mute_valid = false;
    if (dac != selected || muted)
      success = this->apply_mute_(dac, state, true) && success;
  }
  // A requested destination's target must never be written to the old physical route.
  if (selected != nullptr && this->requested_dac_ == this->active_dac && this->fresh_volume_received_ &&
      this->fresh_mute_received_) {
    auto &state = selected == this->tas2780_ ? this->speaker_applied_ : this->lineout_applied_;
    if (!state.volume_valid || state.volume != this->target_volume_ || selected->volume() != this->target_volume_) {
      this->apply_volume_limits_(this->active_dac);
      state.volume_valid = false;
      for (uint8_t attempt = 0; attempt < DAC_WRITE_ATTEMPTS; ++attempt) {
        if (selected->set_volume(this->target_volume_)) {
          state.volume = this->target_volume_;
          state.volume_valid = true;
          break;
        }
      }
      if (!state.volume_valid) {
        state.mute_valid = false;
        ESP_LOGW(TAG, "Selected DAC volume write failed after %u attempts", DAC_WRITE_ATTEMPTS);
        success = false;
      }
    }
    if (success && !muted)
      success = this->apply_mute_(selected, state, false);
  }
  return success;
}

bool DACProxy::receiving_current_output_() const {
  // Reentrant provider publications may still be delivering the previous snapshot.
  return !this->switching_output_ && this->requested_dac_ == this->active_dac &&
         (this->volume_state_ == nullptr || !this->volume_state_->is_publishing());
}

void DACProxy::notify_requested_output_() {
  if (this->volume_state_ != nullptr && this->volume_state_->is_publishing()) {
    this->defer("requested_output", [this]() { this->notify_requested_output_(); });
    return;
  }
  const bool was_switching = this->switching_output_;
  this->switching_output_ = true;
  if (this->volume_state_ != nullptr)
    this->volume_state_->publish_selected_state_(true);
  this->requested_output_callback_.call();
  this->switching_output_ = was_switching;
}

bool DACProxy::request_output(DacOutput output) {
  if (!valid_output(output))
    return false;
  if (this->requested_dac_ == output)
    return true;
  this->requested_dac_ = output;
  this->fresh_volume_received_ = false;
  this->fresh_mute_received_ = false;
  const bool was_switching = this->switching_output_;
  this->switching_output_ = true;
  this->reconcile_();
  if (this->requested_output_notified_)
    this->notify_requested_output_();
  this->switching_output_ = was_switching;
  return true;
}

void DACProxy::select_output_(DacOutput output) {
  if (!this->setup_was_called_ || !valid_output(output))
    return;
  if (this->requested_dac_ != output)
    this->request_output(output);
  if (this->requested_dac_ != output)
    return;
  if (this->switching_output_ || (this->volume_state_ != nullptr && this->volume_state_->is_publishing())) {
    this->pending_output_ = output;
    if (!this->selection_pending_) {
      this->selection_pending_ = true;
      this->defer([this]() {
        this->selection_pending_ = false;
        // Nested route commands coalesce to the latest requested destination.
        if (this->pending_output_ == this->requested_dac_)
          this->select_output_(this->pending_output_);
      });
    }
    return;
  }
  if ((output == SPEAKER ? this->tas2780_ : this->pcm5122_) == nullptr)
    return;
  if (this->active_dac == output) {
    this->reconcile_();
    return;
  }
  this->switching_output_ = true;
  this->fresh_volume_received_ = false;
  this->fresh_mute_received_ = false;
  const bool safely_muted = this->reconcile_();
  if (!safely_muted) {
    ESP_LOGW(TAG, "DAC route safety commit failed");
    this->switching_output_ = false;
    return;
  }
  this->fresh_volume_received_ = false;
  this->fresh_mute_received_ = false;
  this->active_dac = output;
  (output == SPEAKER ? this->speaker_applied_ : this->lineout_applied_) = {};
  // Persist only physical commits, retaining the existing routing preference policy.
  this->restore_state_.dac_output = output;
  if (!this->pref_.save(&this->restore_state_))
    ESP_LOGW(TAG, "Failed to save DAC routing preferences");
  this->send_selected_dac_();
  if (this->volume_state_ != nullptr)
    this->volume_state_->publish_selected_state_(true);
  this->output_change_callback_.call();
  this->selection_callback_.call();
  this->state_callback_.call();
  this->switching_output_ = false;
  this->reconcile_();
}

void DACProxy::activate_line_out() { this->select_output_(LINE_OUT); }
void DACProxy::activate_speaker() { this->select_output_(SPEAKER); }
void DACProxy::activate() {
  if (this->setup_was_called_)
    this->reconcile_();
}

void DACProxy::set_audio_ready(bool ready) {
  const bool was_ready = this->audio_ready_;
  this->audio_ready_ = ready;
  // Hardware may have reset after the suspension mute, even if its cache survived.
  if (!ready || !was_ready) {
    this->speaker_applied_ = {};
    this->lineout_applied_ = {};
  }
  if (this->setup_was_called_ || !ready)
    this->reconcile_();
}

bool DACProxy::force_safety_mute() {
  this->audio_ready_ = false;
  bool success = true;
  for (auto *dac : {this->tas2780_, this->pcm5122_}) {
    if (dac == nullptr)
      continue;
    auto &state = dac == this->tas2780_ ? this->speaker_applied_ : this->lineout_applied_;
    state.mute_valid = false;
    // Bypass the proxy cache: the caller needs acknowledgement of a write now.
    success = this->apply_mute_(dac, state, true) && success;
  }
  return success;
}

bool DACProxy::set_mute_off() {
  if (!this->setup_was_called_)
    return false;
  this->target_muted_ = false;
  this->fresh_mute_received_ = this->receiving_current_output_();
  return this->reconcile_();
}
bool DACProxy::set_mute_on() {
  if (!this->setup_was_called_)
    return false;
  this->target_muted_ = true;
  this->fresh_mute_received_ = this->receiving_current_output_();
  return this->reconcile_();
}
bool DACProxy::is_muted() { return this->target_muted_; }
bool DACProxy::set_volume(float volume) {
  if (!this->setup_was_called_ || !valid_volume(volume))
    return false;
  this->target_volume_ = volume;
  this->fresh_volume_received_ = this->receiving_current_output_();
  return this->reconcile_();
}
float DACProxy::volume() { return this->target_volume_; }

}  // namespace satellite1
}  // namespace esphome
