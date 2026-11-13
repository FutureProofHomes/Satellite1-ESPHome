#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include <functional>

namespace esphome {
namespace tas2780 {

enum ChannelSelect : uint8_t { MONO_DWN_MIX, LEFT_CHANNEL, RIGHT_CHANNEL };

class TAS2780 : public audio_dac::AudioDac, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;

  bool init();
  void reset();
  void activate();
  bool deactivate();
  void set_proxy_managed(bool managed) { this->proxy_managed_ = managed; }
  void set_preparation_guard(void *context, bool (*guard)(void *)) {
    this->preparation_context_ = context;
    this->preparation_guard_ = guard;
  }
  bool activation_failed() const { return this->activation_failed_; }
  void set_activation_guard(std::function<bool()> activation_guard) {
    this->activation_guard_ = std::move(activation_guard);
  }
  bool is_active() const { return this->active_; }
  bool is_activation_pending() const { return this->activation_pending_; }
  void update_register();
  void log_error_states();

  bool set_mute_off() override;
  bool set_mute_on() override;
  bool set_volume(float volume) override;

  bool is_muted() override;
  float volume() override;

  void set_amp_level(uint8_t amp_level) { this->amp_level_ = amp_level; }
  void set_vol_range_min(float min_val) { this->vol_range_min_ = min_val; }
  void set_vol_range_max(float max_val) { this->vol_range_max_ = max_val; }
  void set_selected_channel(ChannelSelect channel) { this->selected_channel_ = channel; }

 protected:
  struct SupplyVoltages {
    uint8_t mode_ctrl;
    float vbat1s;
    float pvdd;
  };

  bool set_power_mode_(const uint8_t power_mode);
  bool activation_allowed_() const;
  void fail_activation_();
  void schedule_activation_finish_();
  bool update_register_checked_();
  void finish_activation_();
  bool read_adc12_(uint8_t msb_reg, uint8_t lsb_reg, uint16_t *raw);
  bool read_supply_voltages_(SupplyVoltages *voltages);
  bool write_mute_();
  bool write_volume_();

  float volume_{0};
  bool active_{false};
  bool activation_pending_{false};
  bool activation_failed_{false};
  bool initialized_{false};
  bool proxy_managed_{false};
  uint32_t activation_generation_{0};
  void *preparation_context_{nullptr};
  bool (*preparation_guard_)(void *){nullptr};
  bool last_supply_sample_valid_{false};
  SupplyVoltages last_supply_voltages_{};
  uint8_t power_mode_{2};
  uint8_t amp_level_{8};
  float vol_range_min_{.3};
  float vol_range_max_{1.};
  ChannelSelect selected_channel_{MONO_DWN_MIX};
  std::function<bool()> activation_guard_{};
};

}  // namespace tas2780
}  // namespace esphome
