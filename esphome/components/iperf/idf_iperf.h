#pragma once

#include <string>

#include "esphome/core/component.h"

namespace esphome {
namespace iperf {

class Iperf : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION + 10; }
  void start_client();

  void set_remote_ip(const std::string &ip) { remote_ip_ = ip; }
  void set_duration(uint32_t duration) { duration_s_ = duration; }

 protected:
  std::string remote_ip_;
  uint16_t port_{5001};
  uint32_t duration_s_{120};
  uint32_t interval_s_{3};
};

}  // namespace iperf
}  // namespace esphome
