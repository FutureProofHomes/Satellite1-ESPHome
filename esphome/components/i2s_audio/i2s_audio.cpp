#include "i2s_audio.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

namespace esphome {
namespace i2s_audio {

  static const char *const TAG = "i2s_audio";

#if defined(USE_ESP_IDF) && (ESP_IDF_VERSION_MAJOR >= 5)
static const uint8_t I2S_NUM_MAX = SOC_I2S_NUM;  // because IDF 5+ took this away :(
#endif

static const size_t DMA_BUFFERS_COUNT = 4;
static const size_t I2S_EVENT_QUEUE_COUNT = DMA_BUFFERS_COUNT + 1;


void I2SAudioComponent::setup() {
  static i2s_port_t next_port_num = I2S_NUM_0;

  if (next_port_num >= I2S_NUM_MAX) {
    ESP_LOGE(TAG, "Too many I2S Audio components!");
    this->mark_failed();
    return;
  }

  this->port_ = next_port_num;
  next_port_num = (i2s_port_t) (next_port_num + 1);

  ESP_LOGCONFIG(TAG, "Setting up I2S Audio...");
}

void I2SAudioComponent::dump_config(){
  esph_log_config(TAG, "I2SController:");
  esph_log_config(TAG, "  AccessMode: %s", this->access_mode_ == I2SAccessMode::DUPLEX ? "duplex" : "exclusive" );
  esph_log_config(TAG, "  Port: %d", this->get_port() );
  if( this->audio_in_ != nullptr ){
    esph_log_config(TAG, "  Reader registered.");
  }
  if( this->audio_out_ != nullptr ){
    esph_log_config(TAG, "  Writer registered.");
  }
}


bool I2SAudioComponent::claim_access_(uint8_t access){
  bool success = false;
  this->lock();
  if( this->access_mode_ == I2SAccessMode::DUPLEX ){
    this->access_state_ |= access;
    success = true;
  }
  else {
    if( this->access_state_ == I2SAccess::FREE ){
      this->access_state_ = access;
    }
    success = this->access_state_ & access;
  }
  this->unlock();
  return success;
}

bool I2SAudioComponent::release_access_(uint8_t access){
  this->lock();
  this->access_state_ = this->access_state_ & (~access);
  this->unlock();
  return true;
}

bool I2SAudioComponent::install_i2s_driver_(i2s_driver_config_t i2s_cfg, uint8_t access){
  bool success = false;
  this->lock();
  esph_log_d(TAG, "Install driver requested by %s", access == I2SAccess::RX ? "Reader" : "Writer");
  if( this->access_state_ == I2SAccess::FREE || this->access_state_ == access ){
    if( this->driver_loaded_ ){
      ESP_LOGW(TAG,"trying to load i2s driver twice");
      return true;
    }
    if(this->access_mode_ == I2SAccessMode::DUPLEX){
      i2s_cfg.mode = (i2s_mode_t) (i2s_cfg.mode | I2S_MODE_TX | I2S_MODE_RX);
    }
    success = ESP_OK == i2s_driver_install(this->get_port(), &i2s_cfg, I2S_EVENT_QUEUE_COUNT, &this->i2s_event_queue_);
    esph_log_d(TAG, "Installing driver : %s", success ? "yes" : "no" );
    i2s_pin_config_t pin_config = this->get_pin_config();
    if( success ){
      this->driver_loaded_ = true;
      if( this->audio_in_ != nullptr )
      {
        pin_config.data_in_num = this->audio_in_->get_din_pin();
      }
      if( this->audio_out_ != nullptr )
      {
        pin_config.data_out_num = this->audio_out_->get_dout_pin();
      }
      success &= ESP_OK == i2s_set_pin(this->get_port(), &pin_config);
      if( success ){
        this->installed_cfg_ = i2s_cfg;
      }
    }
  } else if (this->access_mode_ == I2SAccessMode::DUPLEX && this->driver_loaded_ ){
    success = this->validate_cfg_for_duplex_(i2s_cfg);
    if (!success ){
      ESP_LOGE(TAG, "incompatible i2s settings for duplex mode, access_state: %d", this->access_state_);
    }
  } else {
    ESP_LOGE(TAG, "Unexpected i2s state: mode: %d access_state: %d access_request: %d", (int) this->access_mode_, (int) this->access_state_, (int) access);
  }
  this->unlock();
  return success;
}

bool I2SAudioComponent::uninstall_i2s_driver_(uint8_t access){
  bool success = false;
  this->lock();
  // check that i2s is not occupied by others
  if( (this->access_state_ & access) == access && (this->access_state_ & ~access) == 0 ){
    i2s_zero_dma_buffer(this->get_port());
    esp_err_t err = i2s_driver_uninstall(this->get_port());
    if (err == ESP_OK) {
      success = true;
      this->access_state_ = I2SAccess::FREE;
      this->driver_loaded_ = false;
    } else {
      esph_log_e(TAG, "Couldn't unload driver");
    }
  }
  else {
    // other component hasn't released yet
    // don't uninstall driver, just release caller
    esph_log_d(TAG, "Other component hasn't released");
    this->access_state_ = this->access_state_ & (~access);
  }
  this->unlock();
  return success;
}

void I2SAudioComponent::process_i2s_events(bool &tx_dma_underflow){
  i2s_event_t i2s_event;
      while (xQueueReceive(this->i2s_event_queue_, &i2s_event, 0)) {
        if (i2s_event.type == I2S_EVENT_TX_Q_OVF) {
          tx_dma_underflow = true;
        }
      }
}


bool I2SAudioComponent::validate_cfg_for_duplex_(i2s_driver_config_t& i2s_cfg){
  i2s_driver_config_t& installed = this->installed_cfg_;
  return (
         installed.sample_rate == i2s_cfg.sample_rate
     &&  installed.bits_per_chan == i2s_cfg.bits_per_chan
  );
}


void I2SSettings::dump_i2s_settings() const {
  std::string init_str = this->is_fixed_ ? "Fixed-CFG" : "Initial-CFG";
  if( this->i2s_access_ == I2SAccess::RX ){
    esph_log_config(TAG, "I2S-Reader (%s):", init_str.c_str());
  }
  else{
    esph_log_config(TAG, "I2S-Writer (%s):", init_str.c_str());
  }
  esph_log_config(TAG, "  clk_mode: %s", this->i2s_clk_mode_ == I2S_MODE_MASTER ? "internal" : "external"  );
  esph_log_config(TAG, "  sample-rate: %d bits_per_sample: %d", this->sample_rate_, this->bits_per_sample_ );
  esph_log_config(TAG, "  channel_fmt: %d channels: %d", this->channel_fmt_, this->num_of_channels() );
  esph_log_config(TAG, "  use_apll: %s, use_pdm: %s", this->use_apll_ ? "yes": "no", this->pdm_ ? "yes": "no");
}


i2s_driver_config_t I2SSettings::get_i2s_cfg() const {
  uint8_t mode = this->i2s_clk_mode_ | ( this->i2s_access_ == I2SAccess::RX ? I2S_MODE_RX : I2S_MODE_TX);

  if( this->pdm_){
      mode = (i2s_mode_t) (mode | I2S_MODE_PDM);
  }

  i2s_driver_config_t config = {
      .mode = (i2s_mode_t) mode,
      .sample_rate = this->sample_rate_,
      .bits_per_sample = this->bits_per_sample_,
      .channel_format = this->channel_fmt_,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = DMA_BUFFERS_COUNT,
      .dma_buf_len = 240,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = I2S_PIN_NO_CHANGE,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
#if SOC_I2S_SUPPORTS_TDM
      .chan_mask = I2S_CHANNEL_MONO,
      .total_chan = 0,
      .left_align = false,
      .big_edin = false,
      .bit_order_msb = false,
      .skip_msk = false,
#endif
  };

  return config;
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
