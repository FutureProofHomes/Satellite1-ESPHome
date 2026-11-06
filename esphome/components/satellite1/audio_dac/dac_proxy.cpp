#include "dac_proxy.h"
#include "volume_state.h"

#include <cmath>
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *const TAG = "dac_proxy";
static constexpr uint8_t DAC_WRITE_ATTEMPTS = 3;
static constexpr uint32_t OUTPUT_PREPARATION_TIMEOUT_MS = 2000;

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
    } else if (this->pcm5122_ == nullptr && this->tas2780_ != nullptr)
      this->restore_state_.dac_output = SPEAKER;
    if (!this->pref_.save(&this->restore_state_))
      ESP_LOGW(TAG, "Failed to save DAC routing preferences");
  }
  this->active_dac = static_cast<DacOutput>(this->restore_state_.dac_output);
  this->requested_dac_ = this->active_dac;
  this->output_settings_pref_ =
      global_preferences->make_preference<DACProxyOutputPreferences>(fnv1_hash("satellite1.dac_proxy.outputs.v1"));
  DACProxyOutputPreferences outputs;
  if (this->output_settings_pref_.load(&outputs)) {
    if (outputs.version == 1 && (outputs.enabled_mask & ~3u) == 0)
      this->output_settings_ = outputs;
    else {
      this->output_settings_.enabled_mask = 0;
      this->output_settings_invalid_ = true;
      ESP_LOGW(TAG, "Invalid output settings; outputs disabled until explicitly configured");
    }
  }
  this->activation_pref_ = global_preferences->make_preference<DACProxyActivationPreferences>(
      fnv1_hash("satellite1.dac_proxy.activation.v1"));
  this->activation_state_.requested_mask = 1u << this->requested_dac_;
  DACProxyActivationPreferences activation;
  if (this->activation_pref_.load(&activation)) {
    if (activation.version == 1 && activation.manual <= 1 && (activation.requested_mask & ~3u) == 0)
      this->activation_state_ = activation;
    else {
      this->activation_state_.requested_mask = 0;
      this->activation_state_.manual = 1;
      this->routing_settings_valid_ = false;
      ESP_LOGW(TAG, "Invalid activation settings; outputs inhibited until explicit routing repair");
    }
  }
  this->update_volume_target_();
  this->setup_was_called_ = true;
  this->parent_->add_on_audio_availability_callback([this]() { this->refresh_output(); });
  this->refresh_output();
  this->defer([this]() {
    this->requested_output_notified_ = true;
    this->notify_requested_output_();
  });
}

bool DACProxy::output_configured(DacOutput output) const {
  return valid_output(output) && (output == SPEAKER ? this->tas2780_ : this->pcm5122_) != nullptr;
}

bool DACProxy::output_enabled(DacOutput output) const {
  return this->output_configured(output) && (this->output_settings_.enabled_mask & (1u << output)) != 0;
}

uint8_t DACProxy::effective_outputs() const {
  // Keep missing bindings visible: a restored required DAC must not silently
  // disappear from a dual request and permit partial playback.
  return this->activation_state_.requested_mask & this->output_settings_.enabled_mask;
}

bool DACProxy::update_volume_target_() {
  const auto mask = this->effective_outputs();
  const auto target = (mask & (1u << SPEAKER)) ? SPEAKER : (mask & (1u << LINE_OUT)) ? LINE_OUT : this->requested_dac_;
  const bool changed = target != this->requested_dac_;
  this->requested_dac_ = target;
  return changed;
}

bool DACProxy::apply_activation_(uint8_t requested_mask, bool manual) {
  if (!this->setup_was_called_ || (requested_mask & ~3u) != 0)
    return false;
  if (this->routing_settings_valid_ && this->activation_state_.requested_mask == requested_mask &&
      bool(this->activation_state_.manual) == manual)
    return true;
  auto updated = this->activation_state_;
  updated.requested_mask = requested_mask;
  updated.manual = manual;
  if (!this->activation_pref_.save(&updated)) {
    ESP_LOGW(TAG, "Activation settings save failed; routing unchanged");
    return false;
  }
  const auto previous = this->effective_outputs();
  this->activation_state_ = updated;
  this->routing_settings_valid_ = true;
  const bool target_changed = this->update_volume_target_();
  if (previous != this->effective_outputs() || target_changed)
    this->refresh_output();
  if (target_changed && this->requested_output_notified_)
    this->notify_requested_output_();
  return true;
}

bool DACProxy::set_active_outputs(bool speaker, bool line_out) {
  if ((speaker && !this->output_enabled(SPEAKER)) || (line_out && !this->output_enabled(LINE_OUT)))
    return false;
  return this->apply_activation_((speaker ? 1 : 0) | (line_out ? 2 : 0), true);
}

bool DACProxy::set_automatic_routing(bool jack_connected) {
  if (!this->apply_activation_(1u << (jack_connected ? LINE_OUT : SPEAKER), false))
    return false;
  this->jack_connected_ = jack_connected;
  this->jack_state_known_ = true;
  return true;
}

bool DACProxy::observe_jack_state(bool jack_connected) {
  const bool physical_edge = this->jack_state_known_ && this->jack_connected_ != jack_connected;
  this->jack_connected_ = jack_connected;
  this->jack_state_known_ = true;
  if (physical_edge || this->automatic_routing())
    return this->set_automatic_routing(jack_connected);
  return true;  // An initial sensor publication must not cancel restored manual mode.
}

bool DACProxy::refresh_automatic_routing(bool jack_connected) {
  return !this->automatic_routing() || this->set_automatic_routing(jack_connected);
}

bool DACProxy::set_outputs_enabled(bool speaker, bool line_out) {
  if (!this->setup_was_called_ || (speaker && !this->output_configured(SPEAKER)) ||
      (line_out && !this->output_configured(LINE_OUT)))
    return false;
  DACProxyOutputPreferences updated;
  updated.enabled_mask = (speaker ? 1 : 0) | (line_out ? 2 : 0);
  if (updated.enabled_mask == this->output_settings_.enabled_mask && !this->output_settings_invalid_)
    return true;
  if (this->output_settings_revision_ == UINT32_MAX)
    return false;
  if (!this->output_settings_pref_.save(&updated)) {
    this->output_settings_save_failed_ = true;
    return false;
  }
  const auto previous = this->effective_outputs();
  this->output_settings_ = updated;
  ++this->output_settings_revision_;
  this->output_settings_save_failed_ = false;
  this->output_settings_invalid_ = false;
  const bool target_changed = this->update_volume_target_();
  if (previous != this->effective_outputs() || target_changed)
    this->refresh_output();
  if (target_changed && this->requested_output_notified_)
    this->notify_requested_output_();
  return true;
}

bool DACProxy::speaker_preparation_allowed() const {
  return this->setup_was_called_ && this->parent_ != nullptr && this->parent_->audio_available() &&
         (this->effective_outputs() & (1u << SPEAKER)) != 0 && this->output_phase() == DACOutputPhase::PREPARING;
}

bool DACProxy::invalidate_output_() {
  ++this->transition_generation_;
  this->set_phase_(this->parent_ != nullptr && this->parent_->audio_available() ? DACOutputPhase::INACTIVE
                                                                                : DACOutputPhase::WAITING_FOR_PARENT);
  this->fresh_volume_received_ = this->fresh_mute_received_ = false;
  this->speaker_applied_ = {};
  this->lineout_applied_ = {};
  bool stopped = true;
  if (this->speaker_preparation_ != nullptr)
    stopped = this->speaker_preparation_(this->preparation_context_, DACPreparationCommand::CANCEL) !=
              DACPreparationResult::FAILED;
  return this->reconcile_(false) && stopped;
}

void DACProxy::refresh_output() {
  this->output_dirty_ = true;
  if (this->setup_was_called_)
    this->safe_stop_confirmed_ = this->invalidate_output_();
}

void DACProxy::notify_supply_changed() { this->refresh_output(); }

void DACProxy::fail_output_() {
  this->safe_stop_confirmed_ = this->invalidate_output_();
  this->output_dirty_ = false;
  this->set_phase_(DACOutputPhase::FAILED);
}

void DACProxy::loop() {
  if (!this->setup_was_called_)
    return;
  if (!this->parent_->audio_available()) {
    this->set_phase_(DACOutputPhase::WAITING_FOR_PARENT);
    return;
  }
  if (!this->safe_stop_confirmed_) {
    this->output_dirty_ = false;
    this->set_phase_(DACOutputPhase::FAILED);
    return;
  }
  if (this->effective_outputs() == 0) {
    this->set_phase_(DACOutputPhase::DISABLED);
    return;
  }
  for (const auto output : {SPEAKER, LINE_OUT}) {
    if ((this->effective_outputs() & (1u << output)) && !this->output_configured(output)) {
      this->set_phase_(DACOutputPhase::UNAVAILABLE);
      return;
    }
  }
  const uint32_t generation = this->transition_generation_;
  if (this->output_dirty_) {
    this->output_dirty_ = false;
    if (!this->safe_stop_confirmed_) {
      this->fail_output_();
      return;
    }
    this->attempt_started_ms_ = millis();
    this->set_phase_(DACOutputPhase::PREPARING);
    if (this->effective_outputs() & (1u << SPEAKER)) {
      if (this->speaker_preparation_ == nullptr) {
        this->fail_output_();
        return;
      }
      this->speaker_preparation_(this->preparation_context_, DACPreparationCommand::BEGIN);
      if (generation != this->transition_generation_)
        return;
    } else
      this->commit_output_();
  }
  if (generation != this->transition_generation_)
    return;
  auto phase = this->output_phase();
  if ((phase == DACOutputPhase::PREPARING || phase == DACOutputPhase::SYNCHRONIZING) &&
      uint32_t(millis() - this->attempt_started_ms_) >= OUTPUT_PREPARATION_TIMEOUT_MS) {
    this->fail_output_();
    return;
  }
  if (phase == DACOutputPhase::PREPARING) {
    const auto result = this->speaker_preparation_(this->preparation_context_, DACPreparationCommand::POLL);
    if (generation != this->transition_generation_)
      return;
    if (result == DACPreparationResult::READY)
      this->commit_output_();
    else if (result != DACPreparationResult::PENDING) {
      this->fail_output_();
      return;
    }
  }
  if (generation != this->transition_generation_)
    return;
  phase = this->output_phase();
  if (phase == DACOutputPhase::SYNCHRONIZING && this->fresh_volume_received_ && this->fresh_mute_received_) {
    if (this->reconcile_(true)) {
      if (generation == this->transition_generation_)
        this->set_phase_(DACOutputPhase::READY);
    } else
      this->fail_output_();
  } else if (phase == DACOutputPhase::READY && (this->effective_outputs() & (1u << SPEAKER)) &&
             this->speaker_preparation_(this->preparation_context_, DACPreparationCommand::POLL) !=
                 DACPreparationResult::READY)
    this->fail_output_();
}

void DACProxy::dump_config() {
  ESP_LOGCONFIG(TAG, "  Routing: %s, requested mask=%u, effective mask=%u, committed mask=%u, settings valid=%s",
                this->automatic_routing() ? "automatic" : "manual", this->requested_outputs(),
                this->effective_outputs(), this->active_mask_, YESNO(this->routing_settings_valid_));
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
  if (this->audio_ready() && this->output_active(output) && !this->reconcile_(true)) {
    this->fail_output_();
    // Settings were accepted even if hardware reconciliation failed.
    return true;
  }
  return true;
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

float DACProxy::output_volume_(DacOutput output) const {
  return output == this->active_dac || this->volume_state_ == nullptr ? this->target_volume_
                                                                      : this->volume_state_->output_volume(output);
}

bool DACProxy::reconcile_(bool allow_output) {
  bool success = true;
  const bool permitted = allow_output && this->parent_ != nullptr && this->parent_->audio_available() &&
                         this->requested_dac_ == this->active_dac && this->active_mask_ != 0 &&
                         this->active_mask_ == this->effective_outputs();
  const bool muted = !permitted || this->target_muted_;
  // Driver caches can change before a failed write, so only successful calls confirm applied state.
  for (auto *dac : {this->tas2780_, this->pcm5122_}) {
    if (dac == nullptr)
      continue;
    auto &state = dac == this->tas2780_ ? this->speaker_applied_ : this->lineout_applied_;
    if (state.volume_valid && dac->volume() != state.volume)
      state.volume_valid = false;
    if (state.mute_valid && dac->is_muted() != state.muted)
      state.mute_valid = false;
    const auto output = dac == this->tas2780_ ? SPEAKER : LINE_OUT;
    if (!this->output_active(output) || muted || this->output_volume_(output) < 0.001f)
      success = this->apply_mute_(dac, state, true) && success;
  }
  // No newly requested volume may leak onto the previous committed output set.
  const auto phase = this->output_phase();
  if ((phase == DACOutputPhase::SYNCHRONIZING || phase == DACOutputPhase::READY) && this->parent_->audio_available() &&
      this->active_mask_ == this->effective_outputs() && this->requested_dac_ == this->active_dac &&
      this->fresh_volume_received_ && this->fresh_mute_received_) {
    for (const auto output : {SPEAKER, LINE_OUT}) {
      if (!this->output_active(output))
        continue;
      auto *dac = output == SPEAKER ? this->tas2780_ : this->pcm5122_;
      if (dac == nullptr)
        return false;
      auto &state = output == SPEAKER ? this->speaker_applied_ : this->lineout_applied_;
      const float volume = this->output_volume_(output);
      if (!state.volume_valid || state.volume != volume || dac->volume() != volume) {
        this->apply_volume_limits_(output);
        state.volume_valid = false;
        for (uint8_t attempt = 0; attempt < DAC_WRITE_ATTEMPTS; ++attempt) {
          if (dac->set_volume(volume)) {
            state.volume = volume;
            state.volume_valid = true;
            break;
          }
        }
        if (!state.volume_valid) {
          state.mute_valid = false;
          ESP_LOGW(TAG, "DAC volume write failed after %u attempts", DAC_WRITE_ATTEMPTS);
          success = false;
        }
      }
    }
    // Finish all volume writes before allowing either DAC to become audible.
    if (success && !muted) {
      for (const auto output : {SPEAKER, LINE_OUT}) {
        if (this->output_active(output) && this->output_volume_(output) >= 0.001f) {
          auto *dac = output == SPEAKER ? this->tas2780_ : this->pcm5122_;
          auto &state = output == SPEAKER ? this->speaker_applied_ : this->lineout_applied_;
          if (!this->apply_mute_(dac, state, false)) {
            success = false;
            break;
          }
        }
      }
    }
  }
  return success;
}

bool DACProxy::reconcile_output_volume(DacOutput output) {
  if (!valid_output(output) || !this->output_configured(output))
    return false;
  if (this->audio_ready() && this->output_active(output) && !this->reconcile_(true))
    this->fail_output_();
  return true;  // Logical volume acceptance is separate from hardware success.
}

bool DACProxy::receiving_current_output_() const {
  // Reentrant provider publications may still be delivering the previous snapshot.
  return !this->switching_output_ && this->requested_dac_ == this->active_dac &&
         (this->output_phase() == DACOutputPhase::SYNCHRONIZING || this->output_phase() == DACOutputPhase::READY) &&
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
  if (!this->output_enabled(output))
    return false;
  return this->apply_activation_(1u << output, true);
}

void DACProxy::commit_output_() {
  const auto generation = this->transition_generation_;
  const auto output = this->requested_dac_;
  const auto mask = this->effective_outputs();
  const bool changed = this->active_dac != output || this->active_mask_ != mask;
  this->set_phase_(DACOutputPhase::SWITCHING);
  this->switching_output_ = true;
  this->fresh_volume_received_ = false;
  this->fresh_mute_received_ = false;
  const bool safely_muted = this->reconcile_(false);
  if (!safely_muted) {
    ESP_LOGW(TAG, "DAC route safety commit failed");
    this->switching_output_ = false;
    this->fail_output_();
    return;
  }
  this->fresh_volume_received_ = false;
  this->fresh_mute_received_ = false;
  this->active_dac = output;
  this->active_mask_ = mask;
  this->speaker_applied_ = {};
  this->lineout_applied_ = {};
  // Persist only physical commits, retaining the existing routing preference policy.
  if (changed) {
    this->restore_state_.dac_output = output;
    if (!this->pref_.save(&this->restore_state_))
      ESP_LOGW(TAG, "Failed to save DAC routing preferences");
  }
  this->send_selected_dac_();
  this->switching_output_ = false;
  this->set_phase_(DACOutputPhase::SYNCHRONIZING);
  this->notify_requested_output_();
  if (generation != this->transition_generation_)
    return;
  if (changed || this->setup_notification_pending_) {
    this->setup_notification_pending_ = false;
    this->output_change_callback_.call();
    if (generation != this->transition_generation_)
      return;
    this->selection_callback_.call();
    if (generation != this->transition_generation_)
      return;
    this->state_callback_.call();
  }
}

void DACProxy::activate_line_out() {
  if (this->output_enabled(LINE_OUT))
    this->apply_activation_(this->requested_outputs() | (1u << LINE_OUT), true);
}
void DACProxy::activate_speaker() {
  if (this->output_enabled(SPEAKER))
    this->apply_activation_(this->requested_outputs() | (1u << SPEAKER), true);
}
void DACProxy::activate() { this->refresh_output(); }

bool DACProxy::force_safety_mute() {
  this->safe_stop_confirmed_ = this->invalidate_output_();
  this->output_dirty_ = false;
  bool success = true;
  for (auto *dac : {this->tas2780_, this->pcm5122_}) {
    if (dac == nullptr)
      continue;
    auto &state = dac == this->tas2780_ ? this->speaker_applied_ : this->lineout_applied_;
    state.mute_valid = false;
    // Bypass the proxy cache: the caller needs acknowledgement of a write now.
    success = this->apply_mute_(dac, state, true) && success;
  }
  return success && this->safe_stop_confirmed_;
}

bool DACProxy::set_mute_off() {
  if (!this->setup_was_called_)
    return false;
  this->target_muted_ = false;
  this->fresh_mute_received_ = this->receiving_current_output_();
  const bool success = this->reconcile_(this->output_phase() == DACOutputPhase::READY);
  if (!success)
    this->fail_output_();
  return success;
}
bool DACProxy::set_mute_on() {
  if (!this->setup_was_called_)
    return false;
  this->target_muted_ = true;
  this->fresh_mute_received_ = this->receiving_current_output_();
  const bool success = this->reconcile_(this->output_phase() == DACOutputPhase::READY);
  if (!success)
    this->fail_output_();
  return success;
}
bool DACProxy::is_muted() { return this->target_muted_; }
bool DACProxy::set_volume(float volume) {
  if (!this->setup_was_called_ || !valid_volume(volume))
    return false;
  this->target_volume_ = volume;
  this->fresh_volume_received_ = this->receiving_current_output_();
  const bool success = this->reconcile_(this->output_phase() == DACOutputPhase::READY);
  if (!success)
    this->fail_output_();
  return success;
}
float DACProxy::volume() { return this->target_volume_; }

}  // namespace satellite1
}  // namespace esphome
