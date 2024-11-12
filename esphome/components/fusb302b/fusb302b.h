#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include "pd.h"

namespace esphome {
namespace power_delivery {


enum FUSB302_state_t {
    FUSB302_STATE_UNATTACHED = 0,
    FUSB302_STATE_ATTACHED
};



class FUSB302B : public PowerDelivery, public Component, public i2c::I2CDevice {
public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;
  
  bool send_message_(const PDMsg &msg) override;
  bool read_message_(PDMsg &msg) override;
  void read_status_();

  void set_irq_pin(int irq_pin){this->irq_pin_ = irq_pin;}


typedef union {
    uint8_t bytes[7];
    struct {
      uint8_t status0a;
      uint8_t status1a;
      uint8_t interrupta;
      uint8_t interruptb;
      uint8_t status0;
      uint8_t status1;
      uint8_t interrupt;
    };
  } fusb_status;
  
  fusb_status status_;


public:
  bool cc_line_selection_();
  bool check_cc_line_();
  void fusb_reset_();
  void fusb_hard_reset_();
  
  void check_status_();
  

  FUSB302_state_t state_{FUSB302_STATE_UNATTACHED};

  uint32_t response_timer_{0};
  uint32_t startup_delay_{0};
  int irq_pin_{0};    
};

}
}