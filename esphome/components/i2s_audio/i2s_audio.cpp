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


void I2SPortComponent::setup() {
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

void I2SPortComponent::dump_config(){
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


bool I2SPortComponent::claim_access_(uint8_t access){
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

bool I2SPortComponent::release_access_(uint8_t access){
  this->lock();
  this->access_state_ = this->access_state_ & (~access);
  this->unlock();
  return true;
}

bool I2SPortComponent::install_i2s_driver_(i2s_driver_config_t i2s_cfg, uint8_t access){
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
    ESP_LOGW(TAG, "Driver already loaded, trying to validate duplex mode: %s", success ? "yes" : "no");
    if (!success ){
      ESP_LOGE(TAG, "incompatible i2s settings for duplex mode, access_state: %d", this->access_state_);
    }
  } else {
    ESP_LOGE(TAG, "Unexpected i2s state: mode: %d access_state: %d access_request: %d", (int) this->access_mode_, (int) this->access_state_, (int) access);
  }
  this->unlock();
  return success;
}

bool I2SPortComponent::uninstall_i2s_driver_(uint8_t access){
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

void I2SPortComponent::process_i2s_events(bool &tx_dma_underflow){
  i2s_event_t i2s_event;
      while (xQueueReceive(this->i2s_event_queue_, &i2s_event, 0)) {
        if (i2s_event.type == I2S_EVENT_TX_Q_OVF) {
          tx_dma_underflow = true;
        }
      }
}


bool I2SPortComponent::validate_cfg_for_duplex_(i2s_driver_config_t& i2s_cfg){
  i2s_driver_config_t& installed = this->installed_cfg_;
  return (
         installed.sample_rate == i2s_cfg.sample_rate
     &&  installed.bits_per_chan == i2s_cfg.bits_per_chan
  );
}


void I2SAudioBase::dump_i2s_settings() const {
  std::string init_str = this->is_fixed_ ? "Fixed-CFG" : "Initial-CFG";
  if( this->i2s_access_ == I2SAccess::RX ){
    esph_log_config(TAG, "I2S-Reader (%s):", init_str.c_str());
  }
  else{
    esph_log_config(TAG, "I2S-Writer (%s):", init_str.c_str());
  }
  esph_log_config(TAG, "  clk_mode: %s", this->i2s_mode_ == I2S_MODE_MASTER ? "internal" : "external"  );
  esph_log_config(TAG, "  sample-rate: %d bits_per_sample: %d", this->sample_rate_, this->bits_per_sample_ );
  esph_log_config(TAG, "  channel_fmt: %d channels: %d", this->channel_fmt_, this->num_of_channels() );
  esph_log_config(TAG, "  use_apll: %s", this->use_apll_ ? "yes": "no");
}


bool I2SPortComponent::start_driver_() {

#ifdef USE_I2S_LEGACY
  return true;
  
#else
  i2s_chan_config_t chan_cfg = {
      .id = this->parent_->get_port(),
      .role = this->i2s_role_,
      .dma_desc_num = this->dma_buffers_count_,
      .dma_frame_num = this->dma_buffer_length_,
      .auto_clear = true,
  };
  
  /* Allocate a new TX channel and get the handle of this channel */
  esp_err_t err = i2s_new_channel(&chan_cfg, &this->tx_handle_, &this->rx_handle_);
  if (err != ESP_OK) {
    this->parent_->unlock();
    return err;
  }

  i2s_std_config_t std_cfg = this->get_std_config();
  /* Initialize the channel */
  
  err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
  err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);

  if (err != ESP_OK) {
    i2s_del_channel(this->tx_handle_);
    this->parent_->unlock();
    return err;
  }
  if (this->i2s_event_queue_ == nullptr) {
    this->i2s_event_queue_ = xQueueCreate(1, sizeof(bool));
  }
  const i2s_event_callbacks_t callbacks = {
      .on_send_q_ovf = i2s_overflow_cb,
  };

  i2s_channel_register_event_callback(this->tx_handle_, &callbacks, this);

  /* Before reading data, start the TX channel first */
  i2s_channel_enable(this->tx_handle_);
  if (err != ESP_OK) {
    i2s_del_channel(this->tx_handle_);
    this->parent_->unlock();
  }

#endif
  return true;
}

bool I2SAudioOut::start_i2s_channel_() {
#ifdef USE_I2S_LEGACY
  if (!this->claim_i2s_access()) {
    return false;
  }

  i2s_driver_config_t config = this->get_i2s_cfg();
  if(!this->parent_->install_i2s_driver_(config, I2SAccess::TX))
  {
    this->parent_->release_access_(I2SAccess::TX); 
    return false;
  }
#else
  if( this->parent_tx_handle_ == nullptr ){
    if( this->parent_->driver_started_ )
    {
      ESP_LOGE(TAG, "I2S driver already started, but no TX handle available.");
      return false;
    }
    this->parent_->init_driver_();
  }
  if( this->parent_->tx_handle_ == nullptr ){
    return false;
  }
  i2s_chan_info_t chan_info;
  esp_err_t err = i2s_channel_get_info(this->parent_->tx_handle_, &chan_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get TX channel info: %s", esp_err_to_name(err));
    return false;
  }
  if( chan_info.direction != I2S_CHANNEL_DIR_TX ){
    ESP_LOGE(TAG, "TX channel is not configured for TX direction");
    return false;
  }
  
  if( chan_info.total_dma_buf_size == 0){
     i2s_std_config_t std_cfg = this->get_std_config();
     err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
      if (err != ESP_OK) {
        i2s_del_channel(this->tx_handle_);
        return false;
      }
      const i2s_event_callbacks_t callbacks = {
        .on_send_q_ovf = this->i2s_overflow_cb,
      };
      i2s_channel_register_event_callback(this->tx_handle_, &callbacks, this);
  }
  
  i2s_channel_enable(this->tx_handle_);
  if (err != ESP_OK) {
    i2s_del_channel(this->tx_handle_);
    return false;
  } 
#endif
  return true;
}

bool I2SAudioOut::stop_i2s_channel_() {
#ifdef USE_I2S_LEGACY
  if (!this->parent_->uninstall_i2s_driver_(I2SAccess::TX)) {
    this->parent_->release_access_(I2SAccess::TX);
    return false;
  }
#else
  if( this->parent_tx_handle_ == nullptr ){
    ESP_LOGE(TAG, "Trying to stop I2S-TX channel, but handle is nullptr.");
    return false;
  }
  if( this->parent_->tx_handle_ == nullptr ){
    return false;
  }
  
  esp_err_t err = i2s_channel_disable(this->tx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable TX channel: %s", esp_err_to_name(err));
    return false;
  }
#endif

  return true;
}


bool I2SAudioIn::start_i2s_channel_() {
#ifdef USE_I2S_LEGACY
  if (!this->claim_i2s_access()) {
    return false;
  }

  i2s_driver_config_t config = this->get_i2s_cfg();
  if(!this->parent_->install_i2s_driver_(config, I2SAccess::RX))
  {
    this->parent_->release_access_(I2SAccess::RX); 
    return false;
  }
#else
  if( this->parent_rx_handle_ == nullptr ){
    if( this->parent_->driver_started_ )
    {
      ESP_LOGE(TAG, "I2S driver already started, but no RX handle available.");
      return false;
    }
    this->parent_->init_driver_();
  }
  if( this->parent_->rx_handle_ == nullptr ){
    return false;
  }
  i2s_chan_info_t chan_info;
  esp_err_t err = i2s_channel_get_info(this->parent_->rx_handle_, &chan_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get RX channel info: %s", esp_err_to_name(err));
    return false;
  }
  if( chan_info.direction != I2S_CHANNEL_DIR_RX ){
    ESP_LOGE(TAG, "RX channel is not configured for TX direction");
    return false;
  }
  
  if( chan_info.total_dma_buf_size == 0){
     i2s_std_config_t std_cfg = this->get_std_config();
     err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
      if (err != ESP_OK) {
        i2s_del_channel(this->rx_handle_);
        return false;
      }
      const i2s_event_callbacks_t callbacks = {
        .on_send_q_ovf = this->i2s_overflow_cb,
      };
      i2s_channel_register_event_callback(this->rx_handle_, &callbacks, this);
  }
  
  i2s_channel_enable(this->rx_handle_);
  if (err != ESP_OK) {
    i2s_del_channel(this->rx_handle_);
    return false;
  } 
#endif
  return true;
}

bool I2SAudioIn::stop_i2s_channel_() {
#ifdef USE_I2S_LEGACY
  if (!this->parent_->uninstall_i2s_driver_(I2SAccess::RX)) {
    this->parent_->release_access_(I2SAccess::RX);
    return false;
  }
#else
  if( this->parent_->rx_handle_ == nullptr ){
    ESP_LOGE(TAG, "Trying to stop I2S-RX channel, but handle is nullptr.");
    return false;
  }
  if( this->parent_->rx_handle_ == nullptr ){
    return false;
  }
  
  esp_err_t err = i2s_channel_disable(this->tx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable RX channel: %s", esp_err_to_name(err));
    return false;
  }
#endif

  return true;
}



i2s_driver_config_t I2SAudioBase::get_i2s_cfg() const {
  uint8_t mode = this->i2s_mode_ | ( this->i2s_access_ == I2SAccess::RX ? I2S_MODE_RX : I2S_MODE_TX);

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
