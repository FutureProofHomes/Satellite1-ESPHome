#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tas2780 {

enum ChannelSelect : uint8_t { MONO_DWN_MIX, LEFT_CHANNEL, RIGHT_CHANNEL };

class TAS2780 : public audio_dac::AudioDac, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;

  void init();
  void reset();
  void activate();
  void deactivate();
  bool is_active() const { return this->active_; }
  /// The power mode finish_activation_() selected from the measured supplies: 2 when PVDD carries a
  /// high-voltage USB-PD contract (full output), 0 when running from the 5 V VBAT1S rail (reduced
  /// output). Meaningful while is_active(); between activations it holds the last selection. Read
  /// by the web UI's /api/sat1/amp endpoint from the httpd task - a plain aligned byte, safe to
  /// read cross-task the way web_server reads entity state.
  uint8_t power_mode() const { return this->power_mode_; }
  /// True during the ~100 ms activation window while the SAR ADC settles and the mode above is
  /// still being chosen.
  bool activation_pending() const { return this->activation_pending_; }
  /// How far open the digital volume control is, 0-100: the mapped fraction write_volume_() last
  /// programmed the DVC register from (volume through the configured range), 0 when muted. The
  /// number a person should read as "what the amp is actually being fed", which is why it is the
  /// post-range value rather than the raw slider position.
  int dvc_percent() const {
    if (this->is_muted_)
      return 0;
    const float level = this->volume_ * (this->vol_range_max_ - this->vol_range_min_) + this->vol_range_min_;
    return static_cast<int>(level * 100.0f + 0.5f);
  }
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

  void set_power_mode_(const uint8_t power_mode);
  void finish_activation_();
  bool read_adc12_(uint8_t msb_reg, uint8_t lsb_reg, uint16_t *raw);
  bool read_supply_voltages_(SupplyVoltages *voltages);
  bool write_mute_();
  bool write_volume_();

  float volume_{0};
  bool active_{false};
  bool activation_pending_{false};
  bool last_supply_sample_valid_{false};
  SupplyVoltages last_supply_voltages_{};
  uint8_t power_mode_{2};
  uint8_t amp_level_{8};
  float vol_range_min_{.3};
  float vol_range_max_{1.};
  ChannelSelect selected_channel_{MONO_DWN_MIX};
};

}  // namespace tas2780
}  // namespace esphome
