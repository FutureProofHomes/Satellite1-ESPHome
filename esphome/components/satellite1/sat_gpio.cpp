#include "sat_gpio.h"

namespace esphome {
namespace satellite1 {

static const char *TAG = "Satellite1-GPIOs";

void Satellite1GPIOPin::digital_write(bool value) {
  if (this->port_ != XMOSPort::OUTPUT_A) {
    ESP_LOGE(TAG, "Trying writing to read only port.");
    return;
  }
  uint8_t payload[2] = {this->pin_, value};
  this->parent_->transfer(DC_RESOURCE::GPIO_PORT_OUT_A, GPIO_SERVICER_CMD_SET_PIN, payload, 2);
}

bool Satellite1GPIOPin::digital_read() {
  uint8_t port_value;
  if (!this->parent_->get_cached_dc_status(this->port_register_, &port_value)) {
    return false;
  }
  return !!(port_value & this->pin_mask_) != this->inverted_;
}

}  // namespace satellite1
}  // namespace esphome
