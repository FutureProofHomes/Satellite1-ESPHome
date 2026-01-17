#pragma once

#include "esphome/components/satellite1/satellite1.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace satellite1 {

class Satellite1DoA : public PollingComponent, public Satellite1SPIService {
 public:
  void set_azimuth_sensor(sensor::Sensor *sensor) { this->azimuth_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { this->confidence_sensor_ = sensor; }
  void set_delay_offsets(const std::vector<float> &offsets) { this->delay_offsets_samples_ = offsets; }

  void update() override;

 protected:
  bool check_caps_();
  void send_offsets_();

  sensor::Sensor *azimuth_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  bool caps_checked_{false};
  bool supported_{false};
  bool offsets_supported_{false};
  bool offsets_sent_{false};
  std::vector<float> delay_offsets_samples_{};
};

}  // namespace satellite1
}  // namespace esphome
