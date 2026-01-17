#include "satellite1_doa.h"

#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *TAG = "satellite1_doa";

bool Satellite1DoA::check_caps_() {
  uint8_t payload[2] = {0};
  if (!this->parent_->transfer(DC_RESOURCE::DOA, DC_DOA_CMD::GET_CAPS, payload, sizeof(payload))) {
    return false;
  }

  if (payload[0] != 0xA5) {
    return false;
  }

  this->offsets_supported_ = payload[1] >= 0x02;
  return payload[1] >= 0x01;
}

void Satellite1DoA::send_offsets_() {
  if (this->delay_offsets_samples_.empty()) {
    return;
  }
  if (!this->offsets_supported_) {
    ESP_LOGW(TAG, "Offsets requested but firmware does not support SET_OFFSETS");
    return;
  }
  if (this->delay_offsets_samples_.size() != 4) {
    ESP_LOGW(TAG, "delay_offsets_samples must have 4 entries");
    return;
  }

  uint8_t payload[sizeof(float) * 4];
  memcpy(payload, this->delay_offsets_samples_.data(), sizeof(payload));
  if (!this->parent_->transfer(DC_RESOURCE::DOA, DC_DOA_CMD::SET_OFFSETS, payload, sizeof(payload))) {
    ESP_LOGW(TAG, "Failed to send delay offsets");
  }
}

void Satellite1DoA::update() {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Satellite1 parent not set");
    return;
  }

  if (!this->caps_checked_) {
    this->supported_ = this->check_caps_();
    this->caps_checked_ = true;
    if (!this->supported_) {
      this->status_set_warning();
      ESP_LOGW(TAG, "DoA capability not detected");
      return;
    }
    this->send_offsets_();
    this->offsets_sent_ = true;
  }

  if (!this->supported_) {
    return;
  }

  if (!this->offsets_sent_) {
    this->send_offsets_();
    this->offsets_sent_ = true;
  }

  uint8_t payload[8];
  if (!this->parent_->transfer(DC_RESOURCE::DOA, DC_DOA_CMD::GET_RESULT, payload, sizeof(payload))) {
    this->status_set_warning();
    ESP_LOGW(TAG, "DoA read failed");
    return;
  }

  float azimuth = 0.0f;
  float confidence = 0.0f;
  memcpy(&azimuth, payload, sizeof(float));
  memcpy(&confidence, payload + sizeof(float), sizeof(float));

  if (this->azimuth_sensor_ != nullptr) {
    this->azimuth_sensor_->publish_state(azimuth);
  }
  if (this->confidence_sensor_ != nullptr) {
    this->confidence_sensor_->publish_state(confidence);
  }
  this->status_clear_warning();
}

}  // namespace satellite1
}  // namespace esphome
