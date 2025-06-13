#pragma once


#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/defines.h"
#ifdef USE_I2S_LEGACY
#include <driver/i2s.h>
#else
#include <driver/i2s_std.h>
#endif


namespace esphome {
namespace i2s_audio {

enum class I2SAccessMode  : uint8_t {EXCLUSIVE, DUPLEX};
class I2SAccess {
public:
   static constexpr uint8_t FREE = 0;
   static constexpr uint8_t RX   = 1;
   static constexpr uint8_t TX   = 2;
};

class I2SAudioBase {
public:
 I2SAudioBase(uint8_t access) : i2s_access_(access) {}
#ifdef USE_I2S_LEGACY
  
  void set_channel(i2s_channel_fmt_t channel) { this->channel_fmt_ = channel; }
  void set_bits_per_sample(i2s_bits_per_sample_t bits_per_sample) { this->bits_per_sample_ = bits_per_sample; }
  void set_bits_per_channel(i2s_bits_per_chan_t bits_per_channel) { this->bits_per_channel_ = bits_per_channel; }
  void set_i2s_comm_fmt(i2s_comm_format_t mode) { this->i2s_comm_fmt_ = mode; }
  i2s_driver_config_t get_i2s_cfg(i2s_mode_t i2s_mode) const;
  int num_of_channels() const { 
      return (this->channel_fmt_ == I2S_CHANNEL_FMT_ONLY_RIGHT
           || this->channel_fmt_ == I2S_CHANNEL_FMT_ONLY_LEFT) ? 1 : 2; 
  }
  uint8_t i2s_bits_per_sample() const { return this->bits_per_sample_; }
#else
  void set_slot_mode(i2s_slot_mode_t slot_mode) { this->slot_mode_ = slot_mode; }
  void set_std_slot_mask(i2s_std_slot_mask_t std_slot_mask) { this->std_slot_mask_ = std_slot_mask; }
  void set_slot_bit_width(i2s_slot_bit_width_t slot_bit_width) { this->slot_bit_width_ = slot_bit_width; }
  void set_i2s_comm_fmt(std::string mode) { this->i2s_comm_fmt_ = std::move(mode); }
  int num_of_channels() const {
    return (this->std_slot_mask_ == I2S_STD_SLOT_LEFT || this->std_slot_mask_ == I2S_STD_SLOT_RIGHT) ? 1 : 2;
  }
  uint8_t i2s_bits_per_sample() const { return (uint8_t) this->slot_bit_width_; }
#endif
  void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
  void set_use_apll(uint32_t use_apll) { this->use_apll_ = use_apll; }
  void set_mclk_multiple(i2s_mclk_multiple_t mclk_multiple) { this->mclk_multiple_ = mclk_multiple; }
  void set_pdm(bool pdm) { this->pdm_ = pdm; }

  void dump_i2s_settings() const;
  bool has_fixed_i2s_rate() const { return this->is_fixed_; }
  bool has_fixed_i2s_bitdepth() const { return this->is_fixed_; }

  virtual void register_at_parent() = 0;
  bool validate_for_duplex(I2SAudioBase *other) const {
    return ( 
      (this->i2s_access_ == other->i2s_access_) &&
      (this->sample_rate_ == other->sample_rate_) &&
      (this->mclk_multiple_ == other->mclk_multiple_) &&
#ifdef USE_I2S_LEGACY
      (this->channel_fmt_ == other->channel_fmt_) &&
      (this->bits_per_sample_ == other->bits_per_sample_) &&
      (this->bits_per_channel_ == other->bits_per_channel_)
#else
      (this->slot_mode_ == other->slot_mode_) &&
      (this->std_slot_mask_ == other->std_slot_mask_) &&
      (this->slot_bit_width_ == other->slot_bit_width_)
#endif           
    );
  }

#ifndef USE_I2S_LEGACY  
  i2s_std_clk_config_t get_std_clk_cfg() const {
      return {
        .sample_rate_hz = this->sample_rate_,
#ifdef I2S_CLK_SRC_APLL         
        .clk_src = this->use_appll_ ? I2S_CLK_SRC_APLL : I2S_CLK_SRC_DEFAULT,
#else
        .clk_src = I2S_CLK_SRC_DEFAULT,
#endif
        .mclk_multiple = this->mclk_multiple_,
    };
   }
  i2s_std_slot_config_t get_std_slot_cfg() const {
      i2s_slot_mode_t slot_mode = this->slot_mode_;
      i2s_std_slot_config_t std_slot_cfg;
      if (this->i2s_comm_fmt_ == "std") {
         std_slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) this->slot_bit_width_, slot_mode);
      } else if (this->i2s_comm_fmt_ == "pcm") {
         std_slot_cfg =
         I2S_STD_PCM_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) this->slot_bit_width_, slot_mode);
      } else {
         std_slot_cfg =
            I2S_STD_MSB_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)this->slot_bit_width_, slot_mode);
      }
#ifdef USE_ESP32_VARIANT_ESP32
      // There seems to be a bug on the ESP32 (non-variant) platform where setting the slot bit width higher then the bits
      // per sample causes the audio to play too fast. Setting the ws_width to the configured slot bit width seems to
      // make it play at the correct speed while sending more bits per slot.
      if (this->slot_bit_width_ != I2S_SLOT_BIT_WIDTH_AUTO) {
         std_slot_cfg.ws_width = static_cast<uint32_t>(this->slot_bit_width_);
      }
#else
      std_slot_cfg.slot_bit_width = this->slot_bit_width_;
#endif
      std_slot_cfg.slot_mask = this->std_slot_mask_;
      return std_slot_cfg;
  }
#endif
protected:
   virtual bool start_i2s_channel_() = 0;
   virtual bool stop_i2s_channel_() = 0;

#ifndef USE_I2S_LEGACY
   virtual bool IRAM_ATTR i2s_overflow_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx){ return true; }   
#endif

#ifdef USE_I2S_LEGACY
  i2s_channel_fmt_t channel_fmt_;
  i2s_bits_per_sample_t bits_per_sample_;
  i2s_bits_per_chan_t bits_per_channel_;
  i2s_comm_format_t i2s_comm_fmt_;
#else
  i2s_slot_mode_t slot_mode_;
  i2s_std_slot_mask_t std_slot_mask_;
  i2s_slot_bit_width_t slot_bit_width_;
  std::string i2s_comm_fmt_;
#endif
  uint32_t sample_rate_;
  bool use_apll_;
  i2s_mclk_multiple_t mclk_multiple_;
  uint8_t i2s_access_;

  bool pdm_{false};
  bool is_fixed_{true};
};



class I2SAudioIn;
class I2SAudioOut;


class I2SPortComponent : public Component {
public:
  void setup() override;
  void dump_config() override;

#ifdef USE_I2S_LEGACY
  i2s_pin_config_t get_pin_config() const {
    return {
        .mck_io_num = this->mclk_pin_,
        .bck_io_num = this->bclk_pin_,
        .ws_io_num = this->lrclk_pin_,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
  }
  void process_i2s_events(bool &tx_dma_underflow);
#else
  i2s_std_gpio_config_t get_pin_config() const;
#endif

  void set_mclk_pin(int pin) { this->mclk_pin_ = pin; }
  void set_bclk_pin(int pin) { this->bclk_pin_ = pin; }
  void set_lrclk_pin(int pin) { this->lrclk_pin_ = pin; }

  void lock() { this->lock_.lock(); }
  bool try_lock() { return this->lock_.try_lock(); }
  void unlock() { this->lock_.unlock(); }

  i2s_port_t get_port() const { return this->port_; }
  
  void set_audio_in(I2SAudioIn* comp_in){ this->audio_in_ = comp_in;}
  void set_audio_out(I2SAudioOut* comp_out){ this->audio_out_ = comp_out;}

  void set_access_mode(I2SAccessMode access_mode){this->access_mode_ = access_mode;}
  bool is_exclusive(){return this->access_mode_ == I2SAccessMode::EXCLUSIVE;}

#ifdef USE_I2S_LEGACY  
  void set_i2s_mode(i2s_mode_t mode) { this->i2s_mode_ = mode; }
#else
  void set_i2s_role(i2s_role_t role) { this->i2s_role_ = role; }
  i2s_chan_handle_t get_tx_handle() const { return this->tx_handle_; }
  i2s_chan_handle_t get_rx_handle() const { return this->rx_handle_; }
#endif

protected:
  friend I2SAudioIn;
  friend I2SAudioOut;

  Mutex lock_;
  I2SAccessMode access_mode_{I2SAccessMode::DUPLEX};
  uint8_t access_state_{I2SAccess::FREE};

  bool claim_access_(uint8_t access);
  bool release_access_(uint8_t access);
#ifdef USE_I2S_LEGACY  
  bool install_i2s_driver_(i2s_driver_config_t i2s_cfg, uint8_t access);
  bool uninstall_i2s_driver_(uint8_t access);
  bool validate_cfg_for_duplex_(i2s_driver_config_t& i2s_cfg);
#else
  bool init_driver_(i2s_std_config_t std_cfg);
  bool free_driver_();
#endif
  
  I2SAudioIn *audio_in_{nullptr};
  I2SAudioOut *audio_out_{nullptr};
#ifdef USE_I2S_LEGACY
  int mclk_pin_{I2S_PIN_NO_CHANGE};
  int bclk_pin_{I2S_PIN_NO_CHANGE};
  i2s_mode_t i2s_mode_{};
  i2s_driver_config_t installed_cfg_{};
#else
  i2s_role_t i2s_role_{};
  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};
  int mclk_pin_{I2S_GPIO_UNUSED};
  int bclk_pin_{I2S_GPIO_UNUSED};
  int dout_pin_{I2S_GPIO_UNUSED};
  int din_pin_{I2S_GPIO_UNUSED};
#endif
  int lrclk_pin_;
  i2s_port_t port_{};
  size_t dma_buffer_length_{240};
  uint8_t dma_buffer_count_{4};

  QueueHandle_t i2s_event_queue_;
  bool driver_loaded_{false};
};


class I2SAudioIn : public I2SAudioBase, public Parented<I2SPortComponent> {
public:
  I2SAudioIn() : I2SAudioBase( I2SAccess::RX ) {}

#ifdef USE_I2S_LEGACY
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
#else
  void set_din_pin(int8_t pin) { this->din_pin_ = (gpio_num_t) pin; }
  gpio_num_t get_din_pin() { return this->din_pin_; }
#endif
  
 void register_at_parent() override {
    this->parent_->set_audio_in(this);
 }
protected:
  bool start_i2s_channel_() override;
  bool stop_i2s_channel_() override;
#ifdef USE_I2S_LEGACY
#if SOC_I2S_SUPPORTS_ADC
  adc1_channel_t adc_channel_{ADC1_CHANNEL_MAX};
  bool use_internal_adc_{false};
#endif
  int8_t din_pin_{I2S_PIN_NO_CHANGE};
#else
  gpio_num_t din_pin_{I2S_GPIO_UNUSED};
#endif
};


class I2SAudioOut : public I2SAudioBase, public Parented<I2SPortComponent> {
public:
  I2SAudioOut() : I2SAudioBase(I2SAccess::TX ) {}

#ifdef USE_I2S_LEGACY
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
  void set_i2s_comm_fmt(i2s_comm_format_t mode) { this->i2s_comm_fmt_ = mode; }
#else
  void set_dout_pin(uint8_t pin) { this->dout_pin_ = (gpio_num_t) pin; }
  gpio_num_t get_dout_pin() { return this->dout_pin_; }
#endif
  size_t get_dma_buffer_size_bytes() const { 
   return this->parent_->dma_buffer_length_ * this->num_of_channels() * this->i2s_bits_per_sample() / 8;
  }
  uint8_t get_dma_buffer_count() const { return this->parent_->dma_buffer_count_; }
  
  void register_at_parent() override {
    this->parent_->set_audio_out(this);
  }

  bool is_adjustable() const { return !this->is_fixed_ && this->parent_->is_exclusive(); }

protected:
  bool start_i2s_channel_() override;
  bool stop_i2s_channel_() override;
#ifdef USE_I2S_LEGACY
#if SOC_I2S_SUPPORTS_DAC
  i2s_dac_mode_t internal_dac_mode_{I2S_DAC_CHANNEL_DISABLE};
#endif
  int8_t dout_pin_{I2S_PIN_NO_CHANGE};
  i2s_comm_format_t i2s_comm_fmt_;
#else
  gpio_num_t dout_pin_{I2S_GPIO_UNUSED};
#endif
};



}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
