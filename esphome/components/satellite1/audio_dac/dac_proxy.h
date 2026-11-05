#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/core/component.h"
#include "esphome/components/satellite1/satellite1.h"
#include "esphome/core/preferences.h"
#include "esphome/core/entity_base.h"

#include <array>
#include <functional>

namespace esphome {
namespace satellite1 {

static const uint8_t AUDIO_SERVICER_CMD_SET_DAC = 0x00;

enum DacOutput : uint8_t {
  SPEAKER = 0,
  LINE_OUT,
};

struct DACProxyRestoreState {
  uint8_t dac_output;
  float speaker_volume;
  bool speaker_is_muted;
  float line_out_volume;
  bool line_out_is_muted;
};

struct DACProxyRoutingPreferences {
  uint8_t version{1};
  uint8_t dac_output{LINE_OUT};
};

struct DACProxyActivationPreferences {
  uint8_t version{1};
  uint8_t manual{0};
  uint8_t requested_mask{2};
};

struct DACVolumeLimits {
  float min_db;
  float max_db;
};

struct DACProxyVolumeLimitPreferences {
  uint8_t version{1};
  DACVolumeLimits speaker{-52.5f, 0.0f};
  DACVolumeLimits line_out{-52.5f, 0.0f};
};

struct DACProxyOutputPreferences {
  uint8_t version{1};
  uint8_t enabled_mask{3};
};
enum class DACOutputPhase : uint8_t {
  WAITING_FOR_PARENT,
  INACTIVE,
  PREPARING,
  SWITCHING,
  SYNCHRONIZING,
  READY,
  FAILED,
  DISABLED,
  UNAVAILABLE
};
enum class DACPreparationCommand : uint8_t { BEGIN, POLL, CANCEL };
enum class DACPreparationResult : uint8_t { INACTIVE, PENDING, READY, FAILED };

class DACOutputVolumeState;

class DACProxy : public audio_dac::AudioDac, public Component, public Satellite1SPIService, public EntityBase {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Retains runtime intent and reports reconciliation write success, not playback readiness.
  bool set_mute_off() override;
  bool set_mute_on() override;
  bool set_volume(float volume) override;

  bool is_muted() override;
  float volume() override;
  bool is_setup_complete() const { return this->setup_was_called_; }
  bool has_requested_output_notification() const { return this->requested_output_notified_; }
  DacOutput selected_output() const { return this->active_dac; }
  DacOutput requested_output() const { return this->requested_dac_; }
  // Accepts logical routing before asynchronous hardware preparation; does not physically select it.
  bool request_output(DacOutput output);
  // Explicit manual requests. The per-DAC activate entry points are additive;
  // request_output() remains an exclusive low-level/manual selection.
  bool set_active_outputs(bool speaker, bool line_out);
  bool set_automatic_routing(bool jack_connected);
  bool observe_jack_state(bool jack_connected);
  bool refresh_automatic_routing(bool jack_connected);
  bool automatic_routing() const { return !this->activation_state_.manual; }
  bool routing_settings_valid() const { return this->routing_settings_valid_; }
  uint8_t requested_outputs() const { return this->activation_state_.requested_mask; }
  uint8_t effective_outputs() const;
  bool output_active(DacOutput output) const { return (this->active_mask_ & (1u << output)) != 0; }
  bool reconcile_output_volume(DacOutput output);
  bool set_volume_limits(DacOutput output, float min_db, float max_db);
  void set_volume_limits_callback(DacOutput output, std::function<void(float, float)> callback) {
    this->volume_limits_callbacks_[output] = std::move(callback);
  }
  void refresh_output();
  void notify_supply_changed();
  bool output_configured(DacOutput output) const;
  bool output_enabled(DacOutput output) const;
  // Accepts the complete runtime setting pair. save() acceptance is not verified
  // physical durability and this return value is not a hardware-ready promise.
  bool set_outputs_enabled(bool speaker, bool line_out);
  bool output_settings_save_failed() const { return this->output_settings_save_failed_; }
  bool output_settings_valid() const { return !this->output_settings_invalid_; }
  uint32_t output_settings_revision() const { return this->output_settings_revision_; }
  DACOutputPhase output_phase() const { return this->phase_.load(std::memory_order_acquire); }
  bool speaker_preparation_allowed() const;
  using SpeakerPreparation = DACPreparationResult (*)(void *, DACPreparationCommand);
  void set_speaker_preparation(void *context, SpeakerPreparation callback) {
    this->preparation_context_ = context;
    this->speaker_preparation_ = callback;
  }
  // Closes permission and forces fresh child mute writes; true confirms those calls only.
  bool force_safety_mute();
  bool audio_ready() const {
    return this->phase_.load(std::memory_order_acquire) == DACOutputPhase::READY && this->parent_ != nullptr &&
           this->parent_->audio_available();
  }

  void set_lineout_dac(audio_dac::AudioDac *pcm5122) { this->pcm5122_ = pcm5122; }
  void set_speaker_dac(audio_dac::AudioDac *tas2780) { this->tas2780_ = tas2780; }
  void set_volume_state(DACOutputVolumeState *state) { this->volume_state_ = state; }
  template<typename F> void add_on_state_callback(F &&callback) {
    this->state_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_selection_callback(F &&callback) {
    this->selection_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_output_change_callback(F &&callback) {
    this->output_change_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_requested_output_callback(F &&callback) {
    this->requested_output_callback_.add(std::forward<F>(callback));
  }

  void activate();
  void activate_line_out();
  void activate_speaker();

  DacOutput active_dac{LINE_OUT};

 private:
  struct AppliedState {
    float volume{0.0f};
    bool muted{true};
    bool volume_valid{false};
    bool mute_valid{false};
  };
  bool apply_mute_(audio_dac::AudioDac *dac, AppliedState &state, bool muted);
  float output_volume_(DacOutput output) const;
  bool setup_was_called_{false};
  ESPPreferenceObject pref_;
  DACProxyRoutingPreferences restore_state_{};
  ESPPreferenceObject activation_pref_;
  DACProxyActivationPreferences activation_state_{};
  bool routing_settings_valid_{true};
  bool jack_state_known_{false};
  bool jack_connected_{false};
  uint8_t active_mask_{0};
  bool apply_activation_(uint8_t requested_mask, bool manual);
  bool update_volume_target_();
  ESPPreferenceObject volume_limits_pref_;
  DACProxyVolumeLimitPreferences volume_limits_{};
  ESPPreferenceObject output_settings_pref_;
  DACProxyOutputPreferences output_settings_{};
  uint32_t output_settings_revision_{0};
  bool output_settings_save_failed_{false};
  bool output_settings_invalid_{false};
  std::array<std::function<void(float, float)>, 2> volume_limits_callbacks_;
  void apply_volume_limits_(DacOutput output);
  bool reconcile_(bool allow_output = false);
  bool invalidate_output_();
  void fail_output_();
  void set_phase_(DACOutputPhase phase) { this->phase_.store(phase, std::memory_order_release); }
  bool receiving_current_output_() const;
  void notify_requested_output_();
  void commit_output_();
  std::atomic<DACOutputPhase> phase_{DACOutputPhase::WAITING_FOR_PARENT};
  uint32_t transition_generation_{0};
  uint32_t attempt_started_ms_{0};
  bool output_dirty_{true};
  bool safe_stop_confirmed_{true};
  bool setup_notification_pending_{true};
  void *preparation_context_{nullptr};
  SpeakerPreparation speaker_preparation_{nullptr};
  bool switching_output_{false};
  DacOutput requested_dac_{LINE_OUT};
  bool requested_output_notified_{false};
  bool fresh_volume_received_{false};
  bool fresh_mute_received_{false};
  DACOutputVolumeState *volume_state_{nullptr};
  float target_volume_{0.5f};
  bool target_muted_{false};

  void send_selected_dac_() {}

  audio_dac::AudioDac *pcm5122_{nullptr};
  audio_dac::AudioDac *tas2780_{nullptr};
  AppliedState speaker_applied_{};
  AppliedState lineout_applied_{};

  CallbackManager<void()> output_change_callback_{};
  CallbackManager<void()> requested_output_callback_{};
  CallbackManager<void()> state_callback_{};
  CallbackManager<void()> selection_callback_{};
};

}  // namespace satellite1
}  // namespace esphome
