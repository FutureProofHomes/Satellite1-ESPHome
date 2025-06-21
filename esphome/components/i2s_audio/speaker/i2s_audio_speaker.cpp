#include "i2s_audio_speaker.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esp_timer.h"

namespace esphome {
namespace i2s_audio {

static const size_t TASK_STACK_SIZE = 4096;
static const ssize_t TASK_PRIORITY = 23;

static const char *const TAG = "i2s_audio.speaker";

enum SpeakerEventGroupBits : uint32_t {
  COMMAND_START = (1 << 0),            // starts the speaker task
  COMMAND_STOP = (1 << 1),             // stops the speaker task
  COMMAND_STOP_GRACEFULLY = (1 << 2),  // Stops the speaker task once all data has been written
  STATE_STARTING = (1 << 10),
  STATE_RUNNING = (1 << 11),
  STATE_STOPPING = (1 << 12),
  STATE_STOPPED = (1 << 13),
  ERR_TASK_FAILED_TO_START = (1 << 14),
  ERR_ESP_INVALID_STATE = (1 << 15),
  ERR_ESP_NOT_SUPPORTED = (1 << 16),
  ERR_ESP_INVALID_ARG = (1 << 17),
  ERR_ESP_INVALID_SIZE = (1 << 18),
  ERR_ESP_NO_MEM = (1 << 19),
  ERR_ESP_FAIL = (1 << 20),
  ALL_ERR_ESP_BITS = ERR_ESP_INVALID_STATE | ERR_ESP_NOT_SUPPORTED | ERR_ESP_INVALID_ARG | ERR_ESP_INVALID_SIZE |
                     ERR_ESP_NO_MEM | ERR_ESP_FAIL,
  ALL_BITS = 0x00FFFFFF,  // All valid FreeRTOS event group bits
};

// Translates a SpeakerEventGroupBits ERR_ESP bit to the coressponding esp_err_t
static esp_err_t err_bit_to_esp_err(uint32_t bit) {
  switch (bit) {
    case SpeakerEventGroupBits::ERR_ESP_INVALID_STATE:
      return ESP_ERR_INVALID_STATE;
    case SpeakerEventGroupBits::ERR_ESP_INVALID_ARG:
      return ESP_ERR_INVALID_ARG;
    case SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE:
      return ESP_ERR_INVALID_SIZE;
    case SpeakerEventGroupBits::ERR_ESP_NO_MEM:
      return ESP_ERR_NO_MEM;
    case SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED:
      return ESP_ERR_NOT_SUPPORTED;
    default:
      return ESP_FAIL;
  }
}

/// @brief Multiplies the input array of Q15 numbers by a Q15 constant factor
///
/// Based on `dsps_mulc_s16_ansi` from the esp-dsp library:
/// https://github.com/espressif/esp-dsp/blob/master/modules/math/mulc/fixed/dsps_mulc_s16_ansi.c
/// (accessed on 2024-09-30).
/// @param input Array of Q15 numbers
/// @param output Array of Q15 numbers
/// @param len Length of array
/// @param c Q15 constant factor
static void q15_multiplication(const int16_t *input, int16_t *output, size_t len, int16_t c) {
  for (int i = 0; i < len; i++) {
    int32_t acc = (int32_t) input[i] * (int32_t) c;
    output[i] = (int16_t) (acc >> 15);
  }
}

// Lists the Q15 fixed point scaling factor for volume reduction.
// Has 100 values representing silence and a reduction [49, 48.5, ... 0.5, 0] dB.
// dB to PCM scaling factor formula: floating_point_scale_factor = 2^(-db/6.014)
// float to Q15 fixed point formula: q15_scale_factor = floating_point_scale_factor * 2^(15)
static const std::vector<int16_t> Q15_VOLUME_SCALING_FACTORS = {
    0,     116,   122,   130,   137,   146,   154,   163,   173,   183,   194,   206,   218,   231,   244,
    259,   274,   291,   308,   326,   345,   366,   388,   411,   435,   461,   488,   517,   548,   580,
    615,   651,   690,   731,   774,   820,   868,   920,   974,   1032,  1094,  1158,  1227,  1300,  1377,
    1459,  1545,  1637,  1734,  1837,  1946,  2061,  2184,  2313,  2450,  2596,  2750,  2913,  3085,  3269,
    3462,  3668,  3885,  4116,  4360,  4619,  4893,  5183,  5490,  5816,  6161,  6527,  6914,  7324,  7758,
    8218,  8706,  9222,  9770,  10349, 10963, 11613, 12302, 13032, 13805, 14624, 15491, 16410, 17384, 18415,
    19508, 20665, 21891, 23189, 24565, 26022, 27566, 29201, 30933, 32767};


void I2SAudioSpeaker::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2S Audio Speaker...");

    this->event_group_ = xEventGroupCreate();

  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }

  this->lock_ = xSemaphoreCreateMutex();
}

void I2SAudioSpeaker::loop() {
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  if (event_group_bits & SpeakerEventGroupBits::STATE_STARTING) {
    ESP_LOGD(TAG, "Starting Speaker");
    this->state_ = speaker::STATE_STARTING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_STARTING);
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_RUNNING) {
    ESP_LOGD(TAG, "Started Speaker");
    this->state_ = speaker::STATE_RUNNING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_RUNNING);
    this->status_clear_warning();
    this->status_clear_error();
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_STOPPING) {
    ESP_LOGD(TAG, "Stopping Speaker");
    this->state_ = speaker::STATE_STOPPING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_STOPPING);
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_STOPPED) {
    if (!this->task_created_) {
      ESP_LOGD(TAG, "Stopped Speaker");
      this->state_ = speaker::STATE_STOPPED;
      xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::ALL_BITS);
      this->speaker_task_handle_ = nullptr;
    }
  }

  if (event_group_bits & SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START) {
    this->status_set_error("Failed to start speaker task");
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START);
  }

  if (event_group_bits & SpeakerEventGroupBits::ALL_ERR_ESP_BITS) {
    uint32_t error_bits = event_group_bits & SpeakerEventGroupBits::ALL_ERR_ESP_BITS;
    ESP_LOGW(TAG, "Error writing to I2S: %s", esp_err_to_name(err_bit_to_esp_err(error_bits)));
    this->status_set_warning();
  }

  if (event_group_bits & SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED) {
    this->status_set_error("Failed to adjust I2S bus to match the incoming audio");
    ESP_LOGE(TAG,
             "Incompatible audio format: sample rate = %" PRIu32 ", channels = %" PRIu8 ", bits per sample = %" PRIu8,
             this->audio_stream_info_.get_sample_rate(), this->audio_stream_info_.get_channels(),
             this->audio_stream_info_.get_bits_per_sample());
}

  xEventGroupClearBits(this->event_group_, ALL_ERR_ESP_BITS);
}

void I2SAudioSpeaker::set_volume(float volume) {
  this->volume_ = volume;
#ifdef USE_AUDIO_DAC
  if (this->audio_dac_ != nullptr) {
    if (volume > 0.0) {
      this->audio_dac_->set_mute_off();
    }
    this->audio_dac_->set_volume(volume);
  } else
#endif
  {
    // Fallback to software volume control by using a Q15 fixed point scaling factor
    ssize_t decibel_index = remap<ssize_t, float>(volume, 0.0f, 1.0f, 0, Q15_VOLUME_SCALING_FACTORS.size() - 1);
    this->q15_volume_factor_ = Q15_VOLUME_SCALING_FACTORS[decibel_index];
  }
}

void I2SAudioSpeaker::set_mute_state(bool mute_state) {
  this->mute_state_ = mute_state;
#ifdef USE_AUDIO_DAC
  if (this->audio_dac_) {
    if (mute_state) {
      this->audio_dac_->set_mute_on();
    } else {
      this->audio_dac_->set_mute_off();
  }
  } else
#endif
  {
    if (mute_state) {
      // Fallback to software volume control and scale by 0
      this->q15_volume_factor_ = 0;
    } else {
      // Revert to previous volume when unmuting
      this->set_volume(this->volume_);
    }
  }
}

#if 0
size_t I2SAudioSpeaker::sync_play(const uint8_t *data, size_t length, audio::tv_t desired_playout_time, TickType_t ticks_to_wait) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
    return 0;
  }
  if (this->state_ != speaker::STATE_RUNNING && this->state_ != speaker::STATE_STARTING) {
    this->start();
  }
  
  uint32_t desired_playout_time_ms = desired_playout_time.to_millis();

  size_t bytes_written = 0;
  uint32_t drop_bytes = 0;
  if ((this->state_ == speaker::STATE_RUNNING) && (this->audio_ring_buffer_.use_count() == 1)) {
    
    if (xSemaphoreTake( this->lock_, pdMS_TO_TICKS(TASK_DELAY_MS))) {
        uint32_t playout_in_ms = this->get_unwritten_audio_ms();
        int32_t  delta_ms =  desired_playout_time_ms - (millis() + playout_in_ms);
        
        if( delta_ms > 0 ){
          this->padded_zero_frames_ += this->get_audio_stream_info().ms_to_frames( delta_ms );
        } else if ( delta_ms < - 10 ){
          uint32_t drop_frames = this->get_audio_stream_info().ms_to_frames( -1 * delta_ms );
          drop_bytes = this->get_audio_stream_info().frames_to_bytes(drop_frames);
          drop_bytes = length > drop_bytes ? drop_bytes : length;
        }
        if( length - drop_bytes > 0 ){
          // Temporarily share ownership of the ring buffer so it won't be deallocated while writing
          std::shared_ptr<RingBuffer> temp_ring_buffer = this->audio_ring_buffer_;
          bytes_written = temp_ring_buffer->write_without_replacement((void *) data, length - drop_bytes, ticks_to_wait);
        }
        // static uint32_t last_call = millis();
        // printf( "Speaker ring-buffer: written: %d [%d]ms, it has %d bytes available and %d free\n", bytes_written, millis()- last_call, temp_ring_buffer->available(), temp_ring_buffer->free());
        // last_call = millis();
        xSemaphoreGive(this->lock_);
    } else {
      printf( "play: Couldn't take Semaphore! \n");
    }

  }
  return bytes_written + drop_bytes;
}
#endif

size_t I2SAudioSpeaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
    return 0;
  }
  if (this->state_ != speaker::STATE_RUNNING && this->state_ != speaker::STATE_STARTING) {
    this->start();
  }
#if 1
  if ((this->state_ != speaker::STATE_RUNNING)) { // || (this->audio_ring_buffer_.use_count() == 1)
    // Unable to write data to a running speaker, so delay the max amount of time so it can get ready
    vTaskDelay(ticks_to_wait);
    ticks_to_wait = 0;
  }
#endif
  
size_t bytes_written = 0;
  if ((this->state_ == speaker::STATE_RUNNING) && (this->audio_ring_buffer_.use_count() == 1)) {
    // Only one owner of the ring buffer (the speaker task), so the ring buffer is allocated and no other components are
    // attempting to write to it.

    // Temporarily share ownership of the ring buffer so it won't be deallocated while writing
    std::shared_ptr<RingBuffer> temp_ring_buffer = this->audio_ring_buffer_;
    if( temp_ring_buffer->free() < length ){
      //printf( "Trying to write: %d, free %d, capacity: %d\n", length, temp_ring_buffer->free(), this->ringbuffer_size_ );
      return 0;
    }
    if (xSemaphoreTake( this->lock_, pdMS_TO_TICKS(10))){
      bytes_written = temp_ring_buffer->write_without_replacement((void *) data, length, ticks_to_wait);
      //before this call count bytes as transfer buffer bytes, after this line count bytes as speaker bytes
      this->bytes_in_ringbuffer_ += bytes_written;
      //printf( "from ringbuffer: %d, from local tracking: %d\n", temp_ring_buffer->available(), this->bytes_in_ringbuffer_ );
      xSemaphoreGive(this->lock_);
    }
  }
  return bytes_written;
}

bool I2SAudioSpeaker::has_buffered_data() const {
  if (this->audio_ring_buffer_ != nullptr) {
    return this->audio_ring_buffer_->available() > 0;
  }
  return false;
}

void I2SAudioSpeaker::speaker_task(void *params) {
  I2SAudioSpeaker *this_speaker = (I2SAudioSpeaker *) params;
  this_speaker->task_created_ = true;

  uint32_t event_group_bits =
      xEventGroupWaitBits(this_speaker->event_group_,
                          SpeakerEventGroupBits::COMMAND_START | SpeakerEventGroupBits::COMMAND_STOP |
                              SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY,  // Bit message to read
                          pdTRUE,                                              // Clear the bits on exit
                          pdFALSE,                                             // Don't wait for all the bits,
                          portMAX_DELAY);                                      // Block indefinitely until a bit is set

  if (event_group_bits & (SpeakerEventGroupBits::COMMAND_STOP | SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY)) {
    // Received a stop signal before the task was requested to start
    this_speaker->delete_task_(0);
  }

  xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_STARTING);

  audio::AudioStreamInfo audio_stream_info = this_speaker->audio_stream_info_;

  const size_t dma_buffer_size_bytes = this_speaker->get_dma_buffer_size_bytes();
  const uint8_t dma_buffers_count = this_speaker->get_dma_buffer_count();
  const size_t dma_buffer_duration_ms = audio_stream_info.bytes_to_ms(dma_buffer_size_bytes);
  const size_t task_delay_ms = dma_buffer_duration_ms * dma_buffers_count / 2;
  
  const uint8_t expand_factor = this_speaker->i2s_bits_per_sample() / audio_stream_info.get_bits_per_sample();
  const size_t read_buffer_size = dma_buffer_size_bytes * dma_buffers_count / expand_factor;
  
  // Ensure ring buffer duration is at least the duration of all DMA buffers
  const size_t duration_settings_bytes = audio_stream_info.ms_to_bytes(this_speaker->buffer_duration_ms_);
  const size_t ring_buffer_size = std::max(duration_settings_bytes, read_buffer_size);
  this_speaker->ringbuffer_size_ = ring_buffer_size;
#ifndef USE_I2S_LEGACY
  uint8_t* scaling_buffer = nullptr;
  if (expand_factor > 1) {
    // Allocate a scaling buffer to convert the audio data to the required bits per sample
    // The size of the scaling buffer is the same as the DMA buffer size, but with the expanded bits per sample
    scaling_buffer = new uint8_t[dma_buffer_size_bytes];
  } 
#endif
  
  if (this_speaker->send_esp_err_to_event_group_(this_speaker->allocate_buffers_(read_buffer_size, ring_buffer_size))) {
    // Failed to allocate buffers
    xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::ERR_ESP_NO_MEM);
    this_speaker->delete_task_(read_buffer_size);
  }

  if (!this_speaker->send_esp_err_to_event_group_(this_speaker->start_i2s_driver_(audio_stream_info))) {
    xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_RUNNING);

    bool stop_gracefully = false;
    uint32_t last_data_received_time = millis();
    bool tx_dma_underflow = false;

    this_speaker->accumulated_frames_written_ = 0;
    this_speaker->last_dma_write_ = 0;
    
    // Keep looping if paused, there is no timeout configured, or data was received more recently than the configured
    // timeout
    while (this_speaker->pause_state_ || !this_speaker->timeout_.has_value() ||
           (millis() - last_data_received_time) <= this_speaker->timeout_.value()) {
      event_group_bits = xEventGroupGetBits(this_speaker->event_group_);

      if (event_group_bits & SpeakerEventGroupBits::COMMAND_STOP) {
        xEventGroupClearBits(this_speaker->event_group_, SpeakerEventGroupBits::COMMAND_STOP);
        break;
      }
      if (event_group_bits & SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY) {
        xEventGroupClearBits(this_speaker->event_group_, SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY);
        stop_gracefully = true;
      }

      if (this_speaker->audio_stream_info_ != audio_stream_info) {
        // Audio stream info changed, stop the speaker task so it will restart with the proper settings.
        break;
      }

#ifdef USE_I2S_LEGACY
      this_speaker->parent_->process_i2s_events(tx_dma_underflow);
#endif

      if (this_speaker->pause_state_) {
        // Pause state is accessed atomically, so thread safe
        // Delay so the task can yields, then skip transferring audio data
        delay(task_delay_ms);
        continue;
      }
      // we always write full dma-buffer-sized chunks to the I2S port
      // write zeros if not enough data is available
      std::memset(this_speaker->data_buffer_, 0, read_buffer_size);
#if 1
      if( this_speaker->last_dma_write_ == 0 ){
        size_t bytes_written = 0;
        if( expand_factor == 1) {
            i2s_channel_write(this_speaker->parent_->get_tx_handle(), this_speaker->data_buffer_,
                          read_buffer_size, &bytes_written, 0);
        } else if (expand_factor == 2) {
          std::memset(scaling_buffer, 0, dma_buffer_size_bytes);
          for (uint32_t i = 0; i < dma_buffers_count; ++i) {
            esp_err_t err = i2s_channel_write(
              this_speaker->parent_->get_tx_handle(), 
              scaling_buffer,
              dma_buffer_size_bytes, 
              &bytes_written, 
              0
            );
            if( bytes_written != dma_buffer_size_bytes){
              break;
            }
          }  
        }
        this_speaker->last_dma_write_ = millis();
      }
#endif        
      size_t delay_bytes = 0;
      size_t bytes_read = 0;
      if (xSemaphoreTake(this_speaker->lock_, pdMS_TO_TICKS(10))) {
        delay_bytes = audio_stream_info.frames_to_bytes(this_speaker->padded_zero_frames_);
        xSemaphoreGive(this_speaker->lock_);
      } else {
        continue;
      }
      if( read_buffer_size > delay_bytes ){
          size_t to_read = read_buffer_size - delay_bytes; 
          bytes_read = this_speaker->audio_ring_buffer_->read(
              (void *) this_speaker->data_buffer_, 
              to_read,
              pdMS_TO_TICKS(task_delay_ms)
          );
          if( delay_bytes == 0 && bytes_read < read_buffer_size ){
            printf("underrun: %d, in_buffer: %d\n", bytes_read, this_speaker->bytes_in_ringbuffer_);
          }
          //delay_bytes = 0;
          
      } else {
        // don't read any bytes, write zeros to all DMA buffers instead
        //printf( "pre-delay_bytes %d, buffer_size %d\n", delay_bytes, read_buffer_size);
        delay_bytes = read_buffer_size;
      }
      //printf( "Read bytes: %d, delayed: %d\n", bytes_read, delay_bytes - audio_stream_info.frames_to_bytes(this_speaker->padded_zero_frames_));
      
      
      if ( bytes_read > 0 && (audio_stream_info.get_bits_per_sample() == 16) && (this_speaker->q15_volume_factor_ < INT16_MAX)) {
        // Scale samples by the volume factor in place
        q15_multiplication((int16_t *) this_speaker->data_buffer_, (int16_t *) this_speaker->data_buffer_,
                            bytes_read / sizeof(int16_t), this_speaker->q15_volume_factor_);
      }

      if( bytes_read > 0 ){
        last_data_received_time = millis();
      }
      
      // Write the audio data to a single DMA buffer at a time to reduce latency for the audio duration played
      // callback.
      for (uint32_t i = 0; i < dma_buffers_count; ++i) {
        const size_t bytes_to_write = read_buffer_size / dma_buffers_count;
        size_t bytes_written = 0;
#ifdef USE_I2S_LEGACY
        if ( expand_factor == 1) {
            i2s_write(this_speaker->parent_->get_port(), this_speaker->data_buffer_ + i * bytes_to_write,
                      bytes_to_write, &bytes_written, pdMS_TO_TICKS(dma_buffer_duration_ms * 5));
        } else if ( expand_factor > 1) {
            i2s_write_expand(this_speaker->parent_->get_port(),
                             this_speaker->data_buffer_ + i * bytes_to_write, bytes_to_write,
                           audio_stream_info.get_bits_per_sample(), this_speaker->bits_per_sample_, &bytes_written,
                             pdMS_TO_TICKS(dma_buffer_duration_ms * 5));
        }
#else
        if( expand_factor == 1) {
          i2s_channel_write(this_speaker->parent_->get_tx_handle(), this_speaker->data_buffer_ + i * bytes_to_write,
                          bytes_to_write, &bytes_written, pdMS_TO_TICKS(dma_buffer_duration_ms * 5));
        } else if (expand_factor == 2) {
          int32_t *output = reinterpret_cast<int32_t*>(scaling_buffer);
          int16_t *input  = reinterpret_cast<int16_t*>(this_speaker->data_buffer_ + i * bytes_to_write);
          size_t  samples = bytes_to_write / sizeof(int16_t);
          for (size_t s = 0; s < samples; ++s) {
             output[s] = static_cast<int32_t>(input[s]) << 16;
          }
          esp_err_t err = i2s_channel_write(
            this_speaker->parent_->get_tx_handle(), 
            scaling_buffer,
            samples * sizeof(int32_t), 
            &bytes_written, 
            pdMS_TO_TICKS(dma_buffer_duration_ms * 5)
          );
          bytes_written /= 2;
        }
#endif
        int64_t now = esp_timer_get_time();

        if (bytes_written != bytes_to_write) {
          xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE);
        }
        
        this_speaker->audio_output_callback_(audio_stream_info.bytes_to_frames(bytes_written),
                                               now + dma_buffer_duration_ms * dma_buffers_count * 1000);

        tx_dma_underflow = false;
      }
      
      if(xSemaphoreTake( this_speaker->lock_, pdMS_TO_TICKS(10))) {  
        this_speaker->bytes_in_ringbuffer_ -= bytes_read;
        this_speaker->padded_zero_frames_ -= audio_stream_info.bytes_to_frames(delay_bytes);
        this_speaker->last_dma_write_ = millis();
        xSemaphoreGive(this_speaker->lock_);
      } else {
        printf( "Couldn't receive semaphore, values not updates");
      }
      
      // No data received
      if ( bytes_read == 0 && stop_gracefully) { // && tx_dma_underflow
        break;
      }
    }
    this_speaker->last_dma_write_ = 0;
    xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_STOPPING);

    this_speaker->stop_i2s_channel_();
#ifndef USE_I2S_LEGACY
    if(scaling_buffer != nullptr)
    {
      // Deallocate the scaling buffer if it was allocated
      delete[] scaling_buffer;
    }    
#endif
  }
  this_speaker->delete_task_(read_buffer_size);
}

void I2SAudioSpeaker::start() {
  if (!this->is_ready() || this->is_failed() || this->status_has_error())
    return;
  if ((this->state_ == speaker::STATE_STARTING) || (this->state_ == speaker::STATE_RUNNING))
    return;

  if (!this->task_created_ && (this->speaker_task_handle_ == nullptr)) {
    xTaskCreate(I2SAudioSpeaker::speaker_task, "speaker_task", TASK_STACK_SIZE, (void *) this, TASK_PRIORITY,
                &this->speaker_task_handle_);

  if (this->speaker_task_handle_ != nullptr) {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_START);
  } else {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START);
    }
  }
}

size_t I2SAudioSpeaker::play_silence(size_t length_ms){
  audio::AudioStreamInfo audio_stream_info = this->audio_stream_info_;
  if (xSemaphoreTake( this->lock_, pdMS_TO_TICKS(10))){
    this->padded_zero_frames_ += audio_stream_info.ms_to_frames(length_ms);
    printf( "play silence %d ms, padded zeros: %d\n", length_ms, this->padded_zero_frames_);  
    xSemaphoreGive(this->lock_);
  }
  return this->padded_zero_frames_;
}

uint32_t I2SAudioSpeaker::get_unwritten_audio_ms() const {
  if( this->audio_ring_buffer_ == nullptr ){
    return 0;
  }
  audio::AudioStreamInfo audio_stream_info = this->audio_stream_info_;
  const size_t dma_buffer_duration_ms = this->get_dma_buffer_size_ms();
  const uint8_t dma_buffers_count = this->get_dma_buffer_count();
  uint32_t pending_ms = 0;
  uint32_t pending_frames = 0;
  if (xSemaphoreTake(this->lock_, pdMS_TO_TICKS(10))){
      pending_frames  = this->audio_stream_info_.bytes_to_frames(this->bytes_in_ringbuffer_);
      pending_frames += this->padded_zero_frames_;
      pending_ms = this->audio_stream_info_.frames_to_microseconds(pending_frames) / 1000;
      //pending in DMA buffers
      uint32_t time_delta = (millis() - this->last_dma_write_);
      int32_t in_dma_buffer = dma_buffers_count * dma_buffer_duration_ms - time_delta;
      pending_ms += in_dma_buffer > 0 ? in_dma_buffer : 0;
      //pending_ms += in_dma_buffer;
      //printf( "pending-ringbuffer: %d\n", this->audio_stream_info_.bytes_to_ms(this->bytes_in_ringbuffer_));
      //printf( "pending-zeros: %d\n", this->audio_stream_info_.frames_to_microseconds(this->padded_zero_frames_) / 1000);
      //printf( "dma-buffer: %d (time_delta: %d)\n", in_dma_buffer, time_delta);
      xSemaphoreGive(this->lock_);
  }
  
  return pending_ms;
}


void I2SAudioSpeaker::stop() { this->stop_(false); }

void I2SAudioSpeaker::finish() { this->stop_(true); }

void I2SAudioSpeaker::stop_(bool wait_on_empty) {
  if (this->is_failed())
    return;
  if (this->state_ == speaker::STATE_STOPPED)
    return;

  if (wait_on_empty) {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY);
  } else {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_STOP);
  }
}

bool I2SAudioSpeaker::send_esp_err_to_event_group_(esp_err_t err) {
  switch (err) {
    case ESP_OK:
      return false;
    case ESP_ERR_INVALID_STATE:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_STATE);
      return true;
    case ESP_ERR_INVALID_ARG:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_ARG);
      return true;
    case ESP_ERR_INVALID_SIZE:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE);
      return true;
    case ESP_ERR_NO_MEM:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_NO_MEM);
      return true;
    case ESP_ERR_NOT_SUPPORTED:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED);
      return true;
    default:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_FAIL);
      return true;
  }
}

esp_err_t I2SAudioSpeaker::allocate_buffers_(size_t data_buffer_size, size_t ring_buffer_size) {
  if (this->data_buffer_ == nullptr) {
    // Allocate data buffer for temporarily storing audio from the ring buffer before writing to the I2S bus
    ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
    this->data_buffer_ = allocator.allocate(data_buffer_size);
  }

  if (this->data_buffer_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  if (this->audio_ring_buffer_.use_count() == 0) {
    // Allocate ring buffer. Uses a shared_ptr to ensure it isn't improperly deallocated.
    this->audio_ring_buffer_ = RingBuffer::create(ring_buffer_size);
  }

  if (this->audio_ring_buffer_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

esp_err_t I2SAudioSpeaker::start_i2s_driver_(audio::AudioStreamInfo &audio_stream_info) {
  if (this->has_fixed_i2s_rate() && (this->sample_rate_ != audio_stream_info.get_sample_rate())) {  // NOLINT
    // Can't reconfigure I2S bus, so the sample rate must match the configured value
    return ESP_ERR_NOT_SUPPORTED;
  }
  if ( this->has_fixed_i2s_bitdepth() && 
       this->i2s_bits_per_sample() != audio_stream_info.get_bits_per_sample() &&
       this->i2s_bits_per_sample() != 2 * audio_stream_info.get_bits_per_sample()
     ) {  // NOLINT
    // Can't reconfigure I2S bus, and bit depth must match the configured value
    return ESP_ERR_NOT_SUPPORTED;
  }

  if( !this->start_i2s_channel_() ){
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

void I2SAudioSpeaker::delete_task_(size_t buffer_size) {
  this->audio_ring_buffer_.reset();  // Releases ownership of the shared_ptr

  if (this->data_buffer_ != nullptr) {
    ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
    allocator.deallocate(this->data_buffer_, buffer_size);
    this->data_buffer_ = nullptr;
  }

  xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::STATE_STOPPED);

  this->task_created_ = false;
  vTaskDelete(nullptr);
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
