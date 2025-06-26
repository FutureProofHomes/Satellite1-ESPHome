#pragma once

#include "esphome/core/defines.h"
#ifdef USE_ESP32

#include <driver/i2s.h>
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace i2s_audio {

enum class I2SAccessMode  : uint8_t {EXCLUSIVE, DUPLEX};
class I2SAccess {
public:
   static constexpr uint8_t FREE = 0;
   static constexpr uint8_t RX   = 1;
   static constexpr uint8_t TX   = 2;
};


class I2SReader;
class I2SWriter;
class I2SAudioComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;

  i2s_pin_config_t get_pin_config() const {
    return {
        .mck_io_num = this->mclk_pin_,
        .bck_io_num = this->bclk_pin_,
        .ws_io_num = this->lrclk_pin_,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
  }

  void set_mclk_pin(int pin) { this->mclk_pin_ = pin; }
  void set_bclk_pin(int pin) { this->bclk_pin_ = pin; }
  void set_lrclk_pin(int pin) { this->lrclk_pin_ = pin; }

  void lock() { this->lock_.lock(); }
  bool try_lock() { return this->lock_.try_lock(); }
  void unlock() { this->lock_.unlock(); }

  i2s_port_t get_port() const { return this->port_; }
  void set_audio_in(I2SReader* comp_in){ this->audio_in_ = comp_in;}
  void set_audio_out(I2SWriter* comp_out){ this->audio_out_ = comp_out;}

  void set_access_mode(I2SAccessMode access_mode){this->access_mode_ = access_mode;}
  bool is_exclusive(){return this->access_mode_ == I2SAccessMode::EXCLUSIVE;}

  void process_i2s_events(bool &tx_dma_underflow);

 protected:
  friend I2SReader;
  friend I2SWriter;

  Mutex lock_;
  I2SAccessMode access_mode_{I2SAccessMode::DUPLEX};
  uint8_t access_state_{I2SAccess::FREE};

  bool claim_access_(uint8_t access);
  bool release_access_(uint8_t access);
  bool install_i2s_driver_(i2s_driver_config_t i2s_cfg, uint8_t access);
  bool uninstall_i2s_driver_(uint8_t access);
  bool validate_cfg_for_duplex_(i2s_driver_config_t& i2s_cfg);

  I2SReader *audio_in_{nullptr};
  I2SWriter *audio_out_{nullptr};

  int mclk_pin_{I2S_PIN_NO_CHANGE};
  int bclk_pin_{I2S_PIN_NO_CHANGE};
  int lrclk_pin_;
  i2s_port_t port_{};
  i2s_driver_config_t installed_cfg_{};
  QueueHandle_t i2s_event_queue_;
  bool driver_loaded_{false};
};

class I2SSettings {
public:
  I2SSettings() = default;
  I2SSettings(uint8_t access) : i2s_access_(access) {}

  i2s_driver_config_t get_i2s_cfg() const;
  void dump_i2s_settings() const;
  
  void set_clk_mode(i2s_mode_t clk_mode){ this->i2s_clk_mode_ = clk_mode; } 
  void set_channel(i2s_channel_fmt_t channel_fmt) { this->channel_fmt_ = channel_fmt; }
  void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
  void set_bits_per_sample(i2s_bits_per_sample_t bits_per_sample) { this->bits_per_sample_ = bits_per_sample; }
  void set_bits_per_channel(i2s_bits_per_chan_t bits_per_channel) { this->bits_per_channel_ = bits_per_channel; }
  void set_use_apll(uint32_t use_apll) { this->use_apll_ = use_apll; }
  
  void set_pdm(bool pdm) { this->pdm_ = pdm; }
  void set_fixed_settings(bool is_fixed){ this->is_fixed_ = is_fixed; }
  int num_of_channels() const { return (this->channel_fmt_ == I2S_CHANNEL_FMT_ONLY_RIGHT
   || this->channel_fmt_ == I2S_CHANNEL_FMT_ONLY_LEFT) ? 1 : 2; }
  

protected:
   bool use_apll_{false};
   i2s_bits_per_sample_t bits_per_sample_;
   i2s_bits_per_chan_t bits_per_channel_;
   i2s_channel_fmt_t channel_fmt_;
   i2s_mode_t i2s_clk_mode_{I2S_MODE_MASTER};
   i2s_mode_t i2s_access_mode_;
   
   bool pdm_{false};
   uint32_t sample_rate_;

   bool is_fixed_{false};
   uint8_t i2s_access_;
};


class I2SReader : public I2SSettings, public Parented<I2SAudioComponent> {
public:
   I2SReader() : I2SSettings( I2SAccess::RX ) {}

   bool install_i2s_driver(i2s_driver_config_t i2s_cfg){
      return this->parent_->install_i2s_driver_(i2s_cfg, I2SAccess::RX);}
   bool uninstall_i2s_driver(){ return this->parent_->uninstall_i2s_driver_(I2SAccess::RX);}
   bool claim_i2s_access(){return this->parent_->claim_access_(I2SAccess::RX);}
   bool release_i2s_access(){return this->parent_->release_access_(I2SAccess::RX);}
   bool is_adjustable(){return !this->is_fixed_ && this->parent_->is_exclusive();}
#if SOC_I2S_SUPPORTS_ADC
  void set_adc_channel(adc1_channel_t channel) {
    this->adc_channel_ = channel;
    this->use_internal_adc_ = true;
  }
#endif
   void set_din_pin(int8_t pin) { this->din_pin_ = pin; }
   int8_t get_din_pin() { return this->din_pin_; }

protected:
#if SOC_I2S_SUPPORTS_ADC
   adc1_channel_t adc_channel_{ADC1_CHANNEL_MAX};
   bool use_internal_adc_{false};
#endif
   int8_t din_pin_{I2S_PIN_NO_CHANGE};
};


class I2SWriter : public I2SSettings, public Parented<I2SAudioComponent> {
public:
   I2SWriter() : I2SSettings(I2SAccess::TX ) {}

   bool install_i2s_driver(i2s_driver_config_t i2s_cfg){
      return this->parent_->install_i2s_driver_(i2s_cfg, I2SAccess::TX); }
   bool uninstall_i2s_driver(){ return this->parent_->uninstall_i2s_driver_(I2SAccess::TX);}
   bool claim_i2s_access(){return this->parent_->claim_access_(I2SAccess::TX);}
   bool release_i2s_access(){return this->parent_->release_access_(I2SAccess::TX);}
   bool is_adjustable(){return !this->is_fixed_ && this->parent_->is_exclusive();}

#if SOC_I2S_SUPPORTS_DAC
  void set_internal_dac_mode(i2s_dac_mode_t mode) { this->internal_dac_mode_ = mode; }
#endif

   void set_dout_pin(int8_t pin) { this->dout_pin_ = pin; }
   int8_t get_dout_pin() { return this->dout_pin_; }

protected:
#if SOC_I2S_SUPPORTS_DAC
   i2s_dac_mode_t internal_dac_mode_{I2S_DAC_CHANNEL_DISABLE};
#endif
   int8_t dout_pin_{I2S_PIN_NO_CHANGE};
};




}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
