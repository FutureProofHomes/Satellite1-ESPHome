#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/core/component.h"
#include "esphome/components/satellite1/satellite1.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace satellite1 {

static const uint8_t AUDIO_SERVICER_CMD_SET_DAC = 0x00;

// Full-scale span of each DAC's volume control, in dB: how much quieter volume 0.0 is than 1.0.
// The gain reconciler needs it to work out how much ducking cancels out a given raise.
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

struct DACProxyRestoreState {
  uint8_t dac_output;
  float speaker_volume;
  bool speaker_is_muted;
  float line_out_volume;
  bool line_out_is_muted;
};

class DACProxy : public audio_dac::AudioDac, public Component, public Satellite1SPIService, public EntityBase {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  bool set_mute_off() override;
  bool set_mute_on() override;

  // Two independent writers reach this component on every media volume change, in an order neither
  // controls: the speaker chain defers its I2C work to I2SAudioSpeaker::loop(), while the media
  // player's on_volume automation runs before that loop iteration. Whichever wrote last used to
  // win, so a reconciler that raised the DAC for a voice response was silently overwritten.
  //
  // Splitting the roles makes the order irrelevant: set_volume() records what the media player
  // asked for, set_volume_floor() what the reconciler needs, and the DAC follows the greater of the
  // two. Not an arbitrary tie-break - both are the same remap of a volume onto the DAC's usable
  // range, applied to media_vol and max(media_vol, voice_vol), so the floor is never lower.
  bool set_volume(float volume) override;
  bool set_volume_floor(float floor);

  bool is_muted() override;
  // The level the hardware is actually at, which is what the ducking math has to be judged
  // against. Use volume_floor() to read back the reconciler's own intent.
  float volume() override;
  float volume_floor() const { return this->volume_floor_; }
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
  void save_volume_restore_state_();
  // Pushes max(requested_volume_, volume_floor_) to whichever DAC is live.
  bool apply_volume_();

  // What the media player asked for, shared by both outputs since there is one volume slider behind
  // them. This is what gets persisted, so a reboot comes back at the media level rather than at a
  // raised level that would restore without the ducking that made it safe.
  float requested_volume_{0.5f};
  // Set only by the gain reconciler. Deliberately not persisted: it is a pure function of the media
  // volume and the Voice Override, and the reconciler re-derives it on boot.
  float volume_floor_{0.0f};

  void send_selected_dac_() {}

  audio_dac::AudioDac *pcm5122_{nullptr};
  audio_dac::AudioDac *tas2780_{nullptr};

  CallbackManager<void()> state_callback_{};
};

}  // namespace satellite1
}  // namespace esphome