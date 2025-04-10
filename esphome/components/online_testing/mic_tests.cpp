#include "mic_tests.h"

#include "esphome/core/log.h"
#include "esp_dsp.h"

#include <cinttypes>
#include <cstdio>

namespace esphome {
namespace online_testing {

static const char *const TAG = "mic_tests";


static const size_t SWEEP_LEN = 1024;
static const size_t INPUT_BUFFER_SIZE = SWEEP_LEN * 4;

// Aligned buffers for ESP-DSP
__attribute__((aligned(16))) float mic_f32[INPUT_BUFFER_SIZE];
__attribute__((aligned(16))) float sweep_f32[SWEEP_LEN];
__attribute__((aligned(16))) float corr_out[INPUT_BUFFER_SIZE + SWEEP_LEN - 1];
__attribute__((aligned(16))) float window[SWEEP_LEN];


// Converts int16 to float
static void convert_to_float(const int16_t *src, float *dst, int len) {
    for (int i = 0; i < len; i++) {
        dst[i] = (float)src[i];
    }
}

static bool detect_sweep(const int16_t *mic, const float sweep_norm ) {
    // Convert int16 to float
    convert_to_float(mic, mic_f32, INPUT_BUFFER_SIZE);

    // Sliding window over mic signal
    float max_similarity = 0.0f;
    int best_offset = -1;

    for (int offset = 0; offset <= INPUT_BUFFER_SIZE - SWEEP_LEN; offset++) {
        float dot = 0.0f;
        float norm_mic = 0.0f;

        dsps_dotprod_f32(&mic_f32[offset], sweep_f32, &dot, SWEEP_LEN);
        dsps_dotprod_f32(&mic_f32[offset], &mic_f32[offset], &norm_mic, SWEEP_LEN);

        float similarity = dot / (sweep_norm * sqrtf(norm_mic) + 1e-8f);
        
        if (similarity > max_similarity) {
            max_similarity = similarity;
            best_offset = offset;
        }
    }

    ESP_LOGD(TAG, "Max correlation: %.2f at index %d", max_similarity, best_offset);
    return max_similarity > 0.45f;  // Tune this threshold for your signal

}


float MicTester::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }


void MicTester::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MicTester...");
  this->read_sweep_();
}

bool MicTester::allocate_buffers_() {
  
  ExternalRAMAllocator<int16_t> allocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
  this->input_buffer_ = allocator.allocate(INPUT_BUFFER_SIZE);
  
  //this->input_buffer_ = (int16_t*) heap_caps_malloc(INPUT_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  if (this->input_buffer_ == nullptr) {
    ESP_LOGW(TAG, "Could not allocate input buffer");
    return false;
  }

  return true;
}

void MicTester::clear_buffers_() {
  this->read_pos_ = 0;
  if (this->input_buffer_ != nullptr) {
    memset(this->input_buffer_, 0, INPUT_BUFFER_SIZE * sizeof(int16_t));
  }
  
}

void MicTester::deallocate_buffers_() {
  ExternalRAMAllocator<int16_t> input_deallocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
  input_deallocator.deallocate(this->input_buffer_, INPUT_BUFFER_SIZE);
  //free( this->input_buffer_);
  this->input_buffer_ = nullptr;
}

void MicTester::read_sweep_() {
    const uint8_t *data = this->ref_media_file_->data;
    const size_t len = this->ref_media_file_->length;
    if (len < 2 * SWEEP_LEN) {
        ESP_LOGE(TAG, "Sweep length should be 1024 samples long.  Length: %d", len / 2 );
        this->mark_failed();
        return;
    }
    ESP_LOGD(TAG, "Reading sweep data from file.  Length: %d", len);
    convert_to_float(reinterpret_cast<const int16_t *>(data), sweep_f32, SWEEP_LEN );
    
    // Apply Hann window to sweep
    dsps_wind_hann_f32(window, SWEEP_LEN);
    for (int i = 0; i < SWEEP_LEN; i++) {
        sweep_f32[i] *= window[i];
    }
    
    this->sweep_norm_ = .0f;
    for (int i = 0; i < SWEEP_LEN; i++) {
        this->sweep_norm_ += sweep_f32[i] * sweep_f32[i];
    }
    this->sweep_norm_ = sqrtf(this->sweep_norm_);
}


int MicTester::read_microphone_() {
  size_t bytes_read = 0;
  if (this->mic_->is_running()) {  // Read audio into input buffer
    size_t buffer_bytes = INPUT_BUFFER_SIZE * sizeof(int16_t);
    size_t to_read = buffer_bytes - this->read_pos_;
    to_read = to_read > 512 ? 512 : to_read;
    
    if (to_read == 0 || this->read_pos_ + to_read > buffer_bytes) {
        ESP_LOGE(TAG, "Invalid read position or size");
        return 0;
    }
    uint8_t* buffer = reinterpret_cast<uint8_t*>(this->input_buffer_);
    
    bytes_read = this->mic_->read( reinterpret_cast<int16_t*>(buffer + this->read_pos_), to_read); //, pdMS_TO_TICKS(300)
    this->read_pos_ += bytes_read;
    if ( this->read_pos_ >= INPUT_BUFFER_SIZE * sizeof(int16_t) ) {
        this->read_pos_ = 0;
        return INPUT_BUFFER_SIZE;
    }
} else {
    ESP_LOGD(TAG, "microphone not running");
  }
  return bytes_read;
}

void MicTester::loop() {
  switch (this->state_) {
    case State::IDLE: {
        if (this->continuous_ && this->desired_state_ == State::IDLE) {
        this->idle_trigger_->trigger();
        {
          this->set_state_(State::START_MICROPHONE, State::DETECTING_SWEEP);
        }
      } else {
        this->high_freq_.stop();
      }
      break;
    }
    case State::START_MICROPHONE: {
      ESP_LOGD(TAG, "Starting Microphone");
      if (!this->allocate_buffers_()) {
        this->status_set_error("Failed to allocate buffers");
        return;
      }
      if (this->status_has_error()) {
        this->status_clear_error();
      }
      this->clear_buffers_();

      this->mic_->start();
      this->high_freq_.start();
      this->set_state_(State::STARTING_MICROPHONE);
      break;
    }
    case State::STARTING_MICROPHONE: {
      if (this->mic_->is_running()) {
        this->set_state_(this->desired_state_);
      }
      break;
    }
    case State::DETECTING_SWEEP: {
       size_t samples_available = this->read_microphone_();
        if ( samples_available == INPUT_BUFFER_SIZE ){
            if( detect_sweep(this->input_buffer_, this->sweep_norm_) ){
                this->defer([this]() { this->sweep_detected_trigger_->trigger();  });
            }
        } 
      break;
    }
    case State::STOP_MICROPHONE: {
      if (this->mic_->is_running()) {
        this->mic_->stop();
        this->set_state_(State::STOPPING_MICROPHONE);
      } else {
        this->set_state_(this->desired_state_);
      }
      break;
    }
    case State::STOPPING_MICROPHONE: {
      if (this->mic_->is_stopped()) {
        this->set_state_(this->desired_state_);
      }
      break;
    }
    default:
      break;
  }
}


static const LogString *voice_assistant_state_to_string(State state) {
  switch (state) {
    case State::IDLE:
      return LOG_STR("IDLE");
    case State::START_MICROPHONE:
      return LOG_STR("START_MICROPHONE");
    case State::STARTING_MICROPHONE:
      return LOG_STR("STARTING_MICROPHONE");
    case State::DETECTING_SWEEP:
      return LOG_STR("DETECTING_SWEEP");
    case State::STOP_MICROPHONE:
      return LOG_STR("STOP_MICROPHONE");
    case State::STOPPING_MICROPHONE:
      return LOG_STR("STOPPING_MICROPHONE");
    default:
      return LOG_STR("UNKNOWN");
  }
};

void MicTester::set_state_(State state) {
  State old_state = this->state_;
  this->state_ = state;
  ESP_LOGD(TAG, "State changed from %s to %s", LOG_STR_ARG(voice_assistant_state_to_string(old_state)),
           LOG_STR_ARG(voice_assistant_state_to_string(state)));
}

void MicTester::set_state_(State state, State desired_state) {
  this->set_state_(state);
  this->desired_state_ = desired_state;
  ESP_LOGD(TAG, "Desired state set to %s", LOG_STR_ARG(voice_assistant_state_to_string(desired_state)));
}

void MicTester::failed_to_start() {
  ESP_LOGE(TAG, "Failed to start server. See Home Assistant logs for more details.");
  this->error_trigger_->trigger("failed-to-start", "Failed to start server. See Home Assistant logs for more details.");
  this->set_state_(State::STOP_MICROPHONE, State::IDLE);
}


void MicTester::request_start(bool continuous) {
  if (this->state_ == State::IDLE) {
    this->continuous_ = continuous;
    this->set_state_(State::START_MICROPHONE, State::DETECTING_SWEEP);
  }
}

void MicTester::request_stop() {
  this->continuous_ = false;

  switch (this->state_) {
    case State::IDLE:
      break;
    case State::START_MICROPHONE:
    case State::STARTING_MICROPHONE:
      this->set_state_(State::STOP_MICROPHONE, State::IDLE);
      break;
    case State::DETECTING_SWEEP:
      this->signal_stop_();
      this->set_state_(State::STOP_MICROPHONE, State::IDLE);
      break;
    case State::STOP_MICROPHONE:
    case State::STOPPING_MICROPHONE:
      this->desired_state_ = State::IDLE;
      break;
  }
}

void MicTester::signal_stop_() {

}


}  // namespace voice_assistant
}  // namespace esphome

