#pragma once

#include "../satellite1.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/automation.h"
#include "esphome/core/preferences.h"

namespace esphome::satellite1 {

class MicrophoneMuteSwitch : public switch_::Switch, public Component, public Parented<Satellite1> {
 public:
  void setup() override;
  void loop() override;
  void set_hardware_sensor(binary_sensor::BinarySensor *sensor) { this->hardware_sensor_ = sensor; }
  // A software mute that its caller puts back afterwards (a sign-in window, a peer's tune session).
  // Not saved, so a reboot returns to the person's own choice, and an unmute is not refused while
  // the hardware slider is engaged, since the slider keeps the microphones muted by itself.
  void set_transient(bool muted);
  template<typename F> void add_on_hardware_mute_change_callback(F &&callback) {
    this->hardware_change_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_unmute_rejected_callback(F &&callback) {
    this->unmute_rejected_callback_.add(std::forward<F>(callback));
  }

 protected:
  void write_state(bool state) override;
  ESPPreferenceObject software_preference_;
  binary_sensor::BinarySensor *hardware_sensor_{nullptr};
  HardwareMute last_hardware_{HardwareMute::UNKNOWN};
  CallbackManager<void(bool)> hardware_change_callback_;
  CallbackManager<void()> unmute_rejected_callback_;
};

class HardwareMuteChangeTrigger : public Trigger<bool> {
 public:
  explicit HardwareMuteChangeTrigger(MicrophoneMuteSwitch *parent) {
    parent->add_on_hardware_mute_change_callback([this](bool muted) { this->trigger(muted); });
  }
};

class UnmuteRejectedTrigger : public Trigger<> {
 public:
  explicit UnmuteRejectedTrigger(MicrophoneMuteSwitch *parent) {
    parent->add_on_unmute_rejected_callback([this]() { this->trigger(); });
  }
};

}  // namespace esphome::satellite1
