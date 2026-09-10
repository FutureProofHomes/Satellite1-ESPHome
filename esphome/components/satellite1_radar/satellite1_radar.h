#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/button/button.h"
#include <cstddef>
#include <atomic>
#include <memory>

#include "radar_entities.h"
#include "ld2410_handler.h"
#include "ld2450_handler.h"
#include "radar_tuner_handler.h"

namespace esphome {
namespace satellite1_radar {

enum class RadarType : uint8_t {
  UNKNOWN = 0,
  NONE,
  LD2410,
  LD2450,
};

static const uint32_t DETECT_TIMEOUT_MS = 3000;

class Satellite1Radar : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  bool can_proceed() override { return this->detection_complete_ || this->is_failed(); }

  RadarType get_detected_type() const { return detected_type_; }
  bool is_detection_complete() const { return detection_complete_; }
  void set_radar_type_text_sensor(text_sensor::TextSensor *sensor) { this->radar_type_text_sensor_ = sensor; }

  // --- Radar tuner ---
  RadarTunerHandler &get_tuner_handler() { return tuner_handler_; }
  void set_ld2410_html(const uint8_t *data, size_t len) {
    ld2410_html_gz_ = data;
    ld2410_html_gz_len_ = len;
  }
  void set_ld2450_html(const uint8_t *data, size_t len) {
    ld2450_html_gz_ = data;
    ld2450_html_gz_len_ = len;
  }

  void set_device_class_indices(uint8_t distance, uint8_t illuminance, uint8_t occupancy, uint8_t motion) {
    this->device_class_meta_ = {distance, illuminance, occupancy, motion};
  }

  void set_unit_indices(uint8_t centimeter, uint8_t percent) { this->unit_meta_ = {centimeter, percent}; }

  void set_icon_indices(uint8_t radar, uint8_t chip, uint8_t signal, uint8_t motion_sensor, uint8_t account_multiple,
                        uint8_t account, uint8_t account_arrow_right, uint8_t tune_vertical, uint8_t factory,
                        uint8_t restart, uint8_t database_refresh) {
    this->icon_meta_ = {
        radar,         chip,    signal,  motion_sensor,   account_multiple, account, account_arrow_right,
        tune_vertical, factory, restart, database_refresh};
  }

 protected:
  void process_detection_();
  void finalize_detection_(RadarType type);
  void attach_tuner_();

  RadarType detected_type_{RadarType::UNKNOWN};
  bool detection_started_{false};
  bool detection_complete_{false};
  bool pre_detect_recovery_pending_{false};
  uint32_t pre_detect_recovery_sent_ms_{0};
  uint32_t detect_start_ms_{0};
  static const uint32_t PRE_DETECT_RECOVERY_SETTLE_MS = 20;
  uint8_t detect_ring_buf_[4]{};
  size_t detect_ring_pos_{0};
  DeviceClassMeta device_class_meta_{};
  UnitMeta unit_meta_{};
  IconMeta icon_meta_{};

  text_sensor::TextSensor *radar_type_text_sensor_{nullptr};

  // Protocol handlers (allocated only for detected radar type)
  std::unique_ptr<LD2410Handler> ld2410_{};
  std::unique_ptr<LD2450Handler> ld2450_{};

  // Registered with the shared web server in setup(), before detection has resolved, because
  // handler registration order is what decides URL conflicts and setup() is the only moment we
  // are guaranteed to run ahead of web_server (DATA=800 against WIFI-1=249). The handler answers
  // 404 for whichever radar is not present, which is the same answer it gave when the old server
  // was started after detection.
  RadarTunerHandler tuner_handler_{};
  bool ld2410_engineering_on_{false};
  std::atomic_bool write_config_pending_{false};
  const uint8_t *ld2410_html_gz_{nullptr};
  size_t ld2410_html_gz_len_{0};
  const uint8_t *ld2450_html_gz_{nullptr};
  size_t ld2450_html_gz_len_{0};
};

}  // namespace satellite1_radar
}  // namespace esphome
