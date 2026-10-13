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

struct DACVolumeLimits {
  float min_db;
  float max_db;
};

struct DACProxyVolumeLimitPreferences {
  uint8_t version{1};
  DACVolumeLimits speaker{-52.5f, 0.0f};
  DACVolumeLimits line_out{-52.5f, 0.0f};
};

class DACOutputVolumeState;

class DACProxy : public audio_dac::AudioDac, public Component, public Satellite1SPIService, public EntityBase {
 public:
  void setup() override;
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
  bool set_volume_limits(DacOutput output, float min_db, float max_db);
  void set_volume_limits_callback(DacOutput output, std::function<void(float, float)> callback) {
    this->volume_limits_callbacks_[output] = std::move(callback);
  }
  void set_audio_ready(bool ready);
  // Closes permission and forces fresh child mute writes; true confirms those calls only.
  bool force_safety_mute();
  bool audio_ready() const {
    return this->audio_ready_ && !this->switching_output_ && this->requested_dac_ == this->active_dac &&
           this->fresh_volume_received_ && this->fresh_mute_received_;
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
  bool setup_was_called_{false};
  ESPPreferenceObject pref_;
  DACProxyRoutingPreferences restore_state_{};
  ESPPreferenceObject volume_limits_pref_;
  DACProxyVolumeLimitPreferences volume_limits_{};
  std::array<std::function<void(float, float)>, 2> volume_limits_callbacks_;
  void apply_volume_limits_(DacOutput output);
  bool reconcile_();
  bool receiving_current_output_() const;
  void notify_requested_output_();
  void select_output_(DacOutput output);
  bool audio_ready_{false};
  bool switching_output_{false};
  DacOutput requested_dac_{LINE_OUT};
  DacOutput pending_output_{LINE_OUT};
  bool requested_output_notified_{false};
  bool selection_pending_{false};
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
