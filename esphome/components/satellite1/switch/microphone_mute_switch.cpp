#include "microphone_mute_switch.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::satellite1 {

void MicrophoneMuteSwitch::setup() {
  // The old switch persisted effective state, which cannot recover software
  // intent reliably. A distinct key stores only independently accepted intent.
  this->restore_mode = switch_::SWITCH_RESTORE_DISABLED;
  this->software_preference_ = global_preferences->make_preference<bool>(this->get_object_id_hash() ^ 0x6D69634Du);
  bool software_muted = false;
  this->software_preference_.load(&software_muted);
  this->parent_->microphone_mute().restore_software(software_muted);
  this->loop();
}

void MicrophoneMuteSwitch::write_state(bool state) {
  auto &mute = this->parent_->microphone_mute();
  if (!mute.request_software(state, millis())) {
    this->publish_state(true);
    this->unmute_rejected_callback_.call();
    return;
  }
  if (!this->software_preference_.save(&state))
    ESP_LOGW("satellite1.microphone_mute", "Could not save software microphone-mute preference");
  this->loop();
}

void MicrophoneMuteSwitch::loop() {
  auto &mute = this->parent_->microphone_mute();
  const auto hardware = mute.hardware_state(millis());
  if (hardware != this->last_hardware_) {
    const auto previous = this->last_hardware_;
    this->last_hardware_ = hardware;
    if (hardware == HardwareMute::UNKNOWN)
      this->hardware_sensor_->invalidate_state();
    else
      this->hardware_sensor_->publish_state(hardware == HardwareMute::ON);
    this->publish_state(mute.effectively_muted(millis()));
    // Initial observation and recovery are silent, not physical transitions.
    if (previous != HardwareMute::UNKNOWN && hardware != HardwareMute::UNKNOWN)
      this->hardware_change_callback_.call(hardware == HardwareMute::ON);
  } else {
    this->publish_state(mute.effectively_muted(millis()));
  }
}

}  // namespace esphome::satellite1
