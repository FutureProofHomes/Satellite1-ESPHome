#pragma once

#include "direction_state.h"
#include "esphome/components/i2s_audio/speaker/i2s_audio_speaker.h"
#include "esphome/components/satellite1/satellite1.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome::satellite1_directional_leds {

class DirectionalLEDs : public Component {
 public:
  void dump_config() override;
  void loop() override;
  void set_satellite(satellite1::Satellite1 *satellite) { this->satellite_ = satellite; }
  void set_speaker(i2s_audio::I2SAudioSpeaker *speaker) { this->speaker_ = speaker; }
  void set_enabled(bool enabled);
  void set_phase(bool listening, bool replying);
  bool available() const { return this->enabled_ && this->available_; }
  bool has_listening_direction() const { return this->available() && this->direction_.has_direction(millis()); }
  float listening_position() const { return this->direction_.position(); }
  bool can_reply() const { return this->available() && this->have_reply_direction_; }
  float reply_position() const { return this->reply_position_; }
  float playback_level() const { return this->speaker_->playback_level(); }
  template<typename F> void add_on_availability_callback(F &&callback) {
    this->availability_callback_.add(std::forward<F>(callback));
  }

 protected:
  void set_available_(bool available);
  void update_meter_();
  bool telemetry_fresh_(uint32_t now) const;
  satellite1::Satellite1 *satellite_{nullptr};
  i2s_audio::I2SAudioSpeaker *speaker_{nullptr};
  DirectionState direction_;
  CallbackManager<void()> availability_callback_;
  uint32_t next_poll_{0};
  uint32_t listening_started_at_{0};
  float reply_position_{12};
  uint8_t probe_failures_{0};
  bool enabled_{false};
  bool available_{false};
  bool was_ready_{false};
  bool supported_{false};
  bool listening_{false};
  bool replying_{false};
  bool have_reply_direction_{false};
};

class AvailabilityChangeTrigger : public Trigger<> {
 public:
  explicit AvailabilityChangeTrigger(DirectionalLEDs *parent) {
    parent->add_on_availability_callback([this]() { this->trigger(); });
  }
};

}  // namespace esphome::satellite1_directional_leds
