// esphome_iperf_component.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/defines.h"

namespace esphome {
namespace iperf {

class Iperf : public Component {
public:  
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION + 10; }
  void setup() override {
        ESP_LOGI("iperf", "IperfComponent setup, remote_ip=%s, port=%u",
                remote_ip_.c_str(), port_);
        this->network_initialized_ = false;
  }
  void loop() override;
  void start_server();
  void start_client();
  void stop();

  void set_remote_ip(const std::string &ip) { remote_ip_ = ip; }
  void set_port(uint16_t p) { port_ = p; }
  void set_duration(uint32_t d) { duration_s_ = d; }
  void set_interval(uint32_t i) { interval_s_ = i; }
 
private:
  bool network_initialized_{false};
  std::string remote_ip_;
  uint16_t port_ = 5001;
  uint32_t duration_s_ = 120;
  uint32_t interval_s_ = 3;
  
};

}  // namespace iperf
}  // namespace esphome
