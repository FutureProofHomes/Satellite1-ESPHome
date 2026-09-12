#pragma once

#include "esphome/core/gpio.h"

#include "satellite1.h"

namespace esphome {
namespace satellite1 {

static const uint8_t GPIO_SERVICER_CMD_READ_PORT = 0x00;
static const uint8_t GPIO_SERVICER_CMD_WRITE_PORT = 0x01;
static const uint8_t GPIO_SERVICER_CMD_SET_PIN = 0x02;

enum class XMOSPort : uint8_t {
  INPUT_A = 0,  // buttons
  INPUT_B,      // rotary encoder
  OUTPUT_A      // explorer board LEDs
};

class Satellite1GPIOPin : public GPIOPin, public Satellite1SPIService {
 public:
  void setup() override {};
  void pin_mode(gpio::Flags flags) override {}
  bool digital_read() override;
  void digital_write(bool value) override;

  void set_pin(XMOSPort port, uint8_t pin) {
    this->port_ = port;
    this->pin_ = pin;
    this->pin_mask_ = 1 << pin;
    switch (port) {
      case XMOSPort::INPUT_A:
        this->port_register_ = DC_STATUS_REGISTER::GPIO_PORT_IN_A;
        break;
      case XMOSPort::INPUT_B:
        this->port_register_ = DC_STATUS_REGISTER::GPIO_PORT_IN_B;
        break;
      case XMOSPort::OUTPUT_A:
        this->port_register_ = DC_STATUS_REGISTER::GPIO_PORT_OUT_A;
        break;
    }
  }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { this->flags_ = flags; }
  gpio::Flags get_flags() const { return this->flags_; }

 protected:
  XMOSPort port_;
  uint8_t pin_;
  DC_STATUS_REGISTER::register_id port_register_;
  uint8_t pin_mask_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace satellite1
}  // namespace esphome
