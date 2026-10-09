#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/core/component.h"
#include "esphome/components/satellite1/satellite1.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace satellite1 {

static const uint8_t AUDIO_SERVICER_CMD_SET_DAC = 0x00;

// Full-scale span of each DAC's volume control, in dB: how much quieter volume 0.0 is than 1.0.
// The gain reconciler needs it to turn a difference in level into a mixer gain.
//
//   TAS2780  vol_range_min_ 0.3 leaves 70 of the DVC register's 0.5 dB steps  -> 35.0 dB
//   PCM5122  volume_min_db_ -52.5f up to volume_max_db_ 0.0f                  -> 52.5 dB
//
// Mirrors the driver defaults, since neither driver exposes its own range. Setting tas2780's
// vol_range_min or pcm5122's volume_min_db in YAML means updating the matching constant here.
static constexpr float SPEAKER_VOLUME_SPAN_DB = 35.0f;
static constexpr float LINE_OUT_VOLUME_SPAN_DB = 52.5f;

enum DacOutput : uint8_t {
  SPEAKER = 0,
  LINE_OUT,
};

// Layout kept from the firmware that persisted media volume and mute here, so a saved output
// selection still loads. The two mute flags are written false and never read.
struct DACProxyRestoreState {
  uint8_t dac_output;
  float speaker_volume;
  bool speaker_is_muted;
  float line_out_volume;
  bool line_out_is_muted;
};

// The gain reconciler (audio_gain_reconcile in config/common/voice_assistant.yaml) is the only
// thing that sets the level of the live DAC, through set_level(). Music and announcements share
// this one output, so the DAC sits at the louder of the two levels and each mixer input is scaled
// down to its own; a hardware mute would silence both.
//
// The speaker chain still calls the AudioDac overrides below - the media player hands its volume
// and mute to every pipeline's speaker, and the I2S speaker forwards them here - but they are
// deliberately ignored. Mute on the media player now means "music gain 0", which only the
// reconciler can carry out. The only hardware mutes left are the ones that keep the inactive DAC
// quiet and both DACs quiet until XMOS releases the I2S clocks.
class DACProxy : public audio_dac::AudioDac, public Component, public Satellite1SPIService, public EntityBase {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  bool set_mute_off() override { return true; }
  bool set_mute_on() override { return true; }
  bool set_volume(float volume) override { return true; }

  // The reconciler's level for the live DAC, 0-1 on the same scale as the media player's remap.
  bool set_level(float level);
  float level() const { return this->level_; }

  bool is_muted() override;
  // The level the hardware is actually at.
  float volume() override;
  // dB of attenuation between volume 0.0 and 1.0 on whichever DAC is live. The gain reconciler
  // scales this by the media player's own volume remap to get dB per unit of media volume.
  float volume_span_db() const {
    return this->active_dac == LINE_OUT ? LINE_OUT_VOLUME_SPAN_DB : SPEAKER_VOLUME_SPAN_DB;
  }

  void set_lineout_dac(audio_dac::AudioDac *pcm5122) { this->pcm5122_ = pcm5122; }
  void set_speaker_dac(audio_dac::AudioDac *tas2780) { this->tas2780_ = tas2780; }
  template<typename F> void add_on_state_callback(F &&callback) {
    this->state_callback_.add(std::forward<F>(callback));
  }

  void activate();
  void activate_line_out();
  void activate_speaker();

  DacOutput active_dac{SPEAKER};

 protected:
  bool setup_was_called_{false};
  ESPPreferenceObject pref_;
  DACProxyRestoreState restore_state_;
  void save_restore_state_();
  // Pushes level_ to whichever DAC is live.
  bool apply_level_();

  // Persisted so the DAC comes back at the level it was at, which the reconciler then restates
  // once the media player and the Announcement Volume have restored.
  float level_{0.5f};

  void send_selected_dac_() {}

  audio_dac::AudioDac *pcm5122_{nullptr};
  audio_dac::AudioDac *tas2780_{nullptr};

  CallbackManager<void()> state_callback_{};
};

}  // namespace satellite1
}  // namespace esphome
