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
  /// The power mode a given pair of supply rails calls for: 2 when PVDD carries a high-voltage
  /// USB-PD contract, 0 for a valid external 5 V VBAT1S, -1 when neither rail qualifies. One
  /// ladder shared by finish_activation_() and the loop() reconcile, so the two can never
  /// disagree about what a measurement means.
  int select_power_mode_(const SupplyVoltages &voltages) const;
  bool read_adc12_(uint8_t msb_reg, uint8_t lsb_reg, uint16_t *raw);
  /// `quiet` suppresses the per-sample debug line - the loop() reconcile polls every 5 s, and a
  /// standing debug drumbeat would bury the log; the activation path keeps its sample visible.
  bool read_supply_voltages_(SupplyVoltages *voltages, bool quiet = false);
  bool write_mute_();
  bool write_volume_();

  float volume_{0};
  bool active_{false};
  bool activation_pending_{false};
  bool last_supply_sample_valid_{false};
  SupplyVoltages last_supply_voltages_{};
  /// The loop() reconcile's clock and debounce - see the comment there.
  uint32_t supply_check_ms_{0};
  uint8_t supply_mismatches_{0};
  uint8_t power_mode_{2};
  uint8_t amp_level_{8};
  float vol_range_min_{.3};
  float vol_range_max_{1.};
  ChannelSelect selected_channel_{MONO_DWN_MIX};
};

}  // namespace tas2780
}  // namespace esphome
