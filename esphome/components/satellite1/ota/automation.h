#pragma once
#include "flashing.h"

#include "esphome/core/automation.h"

namespace esphome {
namespace satellite1 {

template<typename... Ts> class SatelliteFlasherAction : public Action<Ts...> {
 public:
  SatelliteFlasherAction(SatelliteFlasher *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, md5_url)
  TEMPLATABLE_VALUE(std::string, md5)
  TEMPLATABLE_VALUE(std::string, url)

  void play(Ts... x) override {
    if (this->md5_url_.has_value()) {
      this->parent_->set_md5_url(this->md5_url_.value(x...));
    }
    if (this->md5_.has_value()) {
      this->parent_->set_md5(this->md5_.value(x...));
    }
    this->parent_->set_url(this->url_.value(x...));

    this->parent_->flash();
    // Normally never reached due to reboot
  }

 protected:
  SatelliteFlasher *parent_;
};

}  // namespace satellite1
}  // namespace esphome