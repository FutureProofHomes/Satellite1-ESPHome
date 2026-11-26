#pragma once
#include "esphome/components/i2s_audio/speaker/level_meter_users.h"
namespace esphome::i2s_audio {
class I2SAudioSpeaker {
 public:
  bool metering{false};
  float level{0.25f};
  LevelMeterUsers meter_users;
  void set_level_meter_enabled(bool enabled, PlaybackLevelUser user = PlaybackLevelUser::DIRECTIONAL_LEDS) {
    this->meter_users.set_enabled(user, enabled);
    this->metering = this->meter_users.enabled();
  }
  float playback_level() const { return this->metering ? this->level : 0.0f; }
};
}  // namespace esphome::i2s_audio
