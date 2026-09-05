#include "mic_tests.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esp_dsp.h"

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#ifdef USE_ESP32
extern "C" {
#include <lwip/inet.h>
#include <sys/socket.h>
#include <unistd.h>
}
#endif

namespace esphome::online_testing {

static const char *const TAG = "online_testing";
static constexpr size_t INPUT_BUFFER_SIZE = 16000 * 2;
static constexpr size_t SWEEP_LEN = 512;
static constexpr size_t MAX_BUFFER_SIZE = SWEEP_LEN * 2;
static constexpr float DETECTION_THRESHOLD = 0.25f;
static constexpr float DETECTION_RESET_THRESHOLD = 0.20f;
static constexpr uint32_t DETECTION_COOLDOWN_MS = 150;

__attribute__((aligned(16))) static float mic_buffer[MAX_BUFFER_SIZE];
__attribute__((aligned(16))) static float sweep_f32[SWEEP_LEN];
__attribute__((aligned(16))) static float window[SWEEP_LEN];
__attribute__((aligned(16))) static float work_buf[SWEEP_LEN];
static size_t mic_write_index = 0;
static size_t mic_filled = 0;

static inline size_t wrap_index(size_t index) { return index % MAX_BUFFER_SIZE; }

static float detect_sweep_streaming(const int16_t *chunk, size_t chunk_len, float sweep_l2_norm) {
  for (size_t i = 0; i < chunk_len; i++) {
    mic_buffer[mic_write_index] = static_cast<float>(chunk[i]);
    mic_write_index = wrap_index(mic_write_index + 1);
  }
  mic_filled = std::min(mic_filled + chunk_len, MAX_BUFFER_SIZE);
  if (mic_filled < SWEEP_LEN || sweep_l2_norm <= 0.0f)
    return 0.0f;

  float max_similarity = 0.0f;
  for (size_t i = 0; i <= mic_filled - SWEEP_LEN; i++) {
    for (size_t j = 0; j < SWEEP_LEN; j++) {
      work_buf[j] = mic_buffer[wrap_index(mic_write_index + MAX_BUFFER_SIZE - mic_filled + i + j)];
    }
    float dot = 0.0f;
    float mic_energy = 0.0f;
    dsps_dotprod_f32(work_buf, sweep_f32, &dot, SWEEP_LEN);
    dsps_dotprod_f32(work_buf, work_buf, &mic_energy, SWEEP_LEN);
    if (mic_energy < 1.0e6f)
      continue;
    const float mic_l2_norm = sqrtf(mic_energy);
    if (mic_l2_norm <= 0.0f)
      continue;
    const float similarity = std::clamp(dot / (sweep_l2_norm * mic_l2_norm), -1.0f, 1.0f);
    max_similarity = std::max(max_similarity, fabsf(similarity));
  }
  return max_similarity;
}

float MicTester::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

void MicTester::setup() {
  if (this->mic_ == nullptr || this->ref_media_file_ == nullptr) {
    ESP_LOGE(TAG, "Microphone and media_file are required");
    this->mark_failed();
    return;
  }
  this->read_sweep_();
  this->udp_packet_buffer_.resize(this->udp_stream_packet_samples_);
}

bool MicTester::allocate_buffers_() {
  if (this->input_buffer_ != nullptr)
    return true;
  ExternalRAMAllocator<int16_t> allocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
  this->input_buffer_ = allocator.allocate(INPUT_BUFFER_SIZE);
  if (this->input_buffer_ == nullptr) {
    ESP_LOGW(TAG, "Could not allocate input buffer");
    return false;
  }
  this->input_buffer_size_ = INPUT_BUFFER_SIZE;
  return true;
}

void MicTester::clear_buffers_() {
  this->write_pos_ = this->read_pos_ = 0;
  this->energy_accumulator_ = 0.0f;
  this->energy_sample_count_ = 0;
  this->sweep_armed_ = true;
  this->last_sweep_ms_ = 0;
  this->reset_udp_packet_();
}

void MicTester::deallocate_buffers_() {
  if (this->input_buffer_ == nullptr)
    return;
  ExternalRAMAllocator<int16_t> allocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
  allocator.deallocate(this->input_buffer_, INPUT_BUFFER_SIZE);
  this->input_buffer_ = nullptr;
  this->input_buffer_size_ = 0;
}

void MicTester::read_sweep_() {
  const uint8_t *data = this->ref_media_file_->data;
  size_t length = this->ref_media_file_->length;
  // audio_file embeds full WAV files; retain support for the legacy raw PCM payload too.
  if (length >= 12 && std::memcmp(data, "RIFF", 4) == 0 && std::memcmp(data + 8, "WAVE", 4) == 0) {
    size_t offset = 12;
    while (offset + 8 <= length) {
      const uint32_t chunk_size = static_cast<uint32_t>(data[offset + 4]) | (static_cast<uint32_t>(data[offset + 5]) << 8) |
                                  (static_cast<uint32_t>(data[offset + 6]) << 16) | (static_cast<uint32_t>(data[offset + 7]) << 24);
      if (std::memcmp(data + offset, "data", 4) == 0) {
        data += offset + 8;
        length = std::min<size_t>(chunk_size, length - offset - 8);
        break;
      }
      offset += 8 + chunk_size + (chunk_size & 1);
    }
  }
  if (length < sizeof(int16_t) * SWEEP_LEN) {
    ESP_LOGE(TAG, "Sweep needs at least %u PCM samples", SWEEP_LEN);
    this->mark_failed();
    return;
  }
  const int16_t *samples = reinterpret_cast<const int16_t *>(data);
  dsps_wind_hann_f32(window, SWEEP_LEN);
  this->sweep_norm_ = 0.0f;
  for (size_t i = 0; i < SWEEP_LEN; i++) {
    sweep_f32[i] = static_cast<float>(samples[i]) * window[i];
    this->sweep_norm_ += sweep_f32[i] * sweep_f32[i];
  }
  this->sweep_norm_ = sqrtf(this->sweep_norm_);
}

void MicTester::on_audio_data_(const std::vector<uint8_t> &data) {
  if (this->state_ != State::DETECTING_SWEEP || this->input_buffer_ == nullptr)
    return;
  const auto stream_info = this->mic_->get_audio_stream_info();
  const size_t bytes_per_sample = stream_info.samples_to_bytes(1);
  const size_t channels = stream_info.get_channels();
  if (bytes_per_sample == 0 || this->channel_ >= channels)
    return;
  const size_t frame_size = bytes_per_sample * channels;
  const size_t frames = data.size() / frame_size;
  size_t write_pos = this->write_pos_;
  for (size_t i = 0; i < frames; i++) {
    const int32_t sample_q31 = audio::unpack_audio_sample_to_q31(&data[i * frame_size + this->channel_ * bytes_per_sample], bytes_per_sample);
    const int16_t sample = static_cast<int16_t>(sample_q31 >> 13);
    this->input_buffer_[write_pos % this->input_buffer_size_] = sample;
    this->append_udp_sample_(sample);
    this->energy_accumulator_ += static_cast<float>(sample) * sample;
    this->energy_sample_count_++;
    write_pos++;
  }
  this->write_pos_ = write_pos;
}

void MicTester::loop() {
  switch (this->state_) {
    case State::IDLE:
      if (this->continuous_ && this->desired_state_ == State::IDLE) {
        this->set_state_(State::START_MICROPHONE, State::DETECTING_SWEEP);
      } else {
        this->high_freq_.stop();
      }
      break;
    case State::START_MICROPHONE:
      if (!this->allocate_buffers_()) {
        this->status_set_error(LOG_STR("Failed to allocate buffers"));
        return;
      }
      this->status_clear_error();
      this->clear_buffers_();
      mic_write_index = mic_filled = 0;
      if (!this->callback_registered_) {
        this->mic_->add_data_callback([this](const std::vector<uint8_t> &data) { this->on_audio_data_(data); });
        this->callback_registered_ = true;
      }
      this->mic_->start();
      this->high_freq_.start();
      this->set_state_(State::STARTING_MICROPHONE);
      break;
    case State::STARTING_MICROPHONE:
      if (this->mic_->is_running())
        this->set_state_(this->desired_state_);
      break;
    case State::DETECTING_SWEEP: {
      const size_t write_pos = this->write_pos_;
      const size_t read_pos = this->read_pos_;
      if (write_pos <= read_pos || write_pos - read_pos < 480)
        break;
      const size_t count = std::min(write_pos - read_pos, static_cast<size_t>(960));
      int16_t samples[960];
      for (size_t i = 0; i < count; i++)
        samples[i] = this->input_buffer_[(read_pos + i) % this->input_buffer_size_];
      this->read_pos_ = read_pos + count;
      const float correlation = detect_sweep_streaming(samples, count, this->sweep_norm_);
      if (correlation < DETECTION_RESET_THRESHOLD)
        this->sweep_armed_ = true;
      const uint32_t now = millis();
      if (this->sweep_armed_ && correlation > DETECTION_THRESHOLD &&
          now - this->last_sweep_ms_ > DETECTION_COOLDOWN_MS) {
        this->last_sweep_ms_ = now;
        this->sweep_armed_ = false;
        ESP_LOGI(TAG, "Sweep detected: correlation %.2f on channel %u", correlation, this->channel_);
        this->sweep_detected_trigger_->trigger();
      }
      break;
    }
    case State::STOP_MICROPHONE:
      if (this->mic_->is_running()) {
        this->mic_->stop();
        this->set_state_(State::STOPPING_MICROPHONE);
      } else {
        this->set_state_(this->desired_state_);
      }
      break;
    case State::STOPPING_MICROPHONE:
      if (this->mic_->is_stopped()) {
        this->deallocate_buffers_();
        this->set_state_(this->desired_state_);
      }
      break;
    case State::DETECTION_PAUSED:
      break;
  }
}

void MicTester::set_state_(State state) {
  this->state_ = state;
}

void MicTester::set_state_(State state, State desired_state) {
  this->set_state_(state);
  this->desired_state_ = desired_state;
}

void MicTester::failed_to_start() {
  ESP_LOGE(TAG, "Failed to start microphone testing");
  this->set_state_(State::STOP_MICROPHONE, State::IDLE);
}

void MicTester::request_start(bool continuous) {
  this->continuous_ = continuous;
  if (this->state_ == State::IDLE) {
    this->set_state_(State::START_MICROPHONE, State::DETECTING_SWEEP);
  } else if (this->state_ == State::STOP_MICROPHONE || this->state_ == State::STOPPING_MICROPHONE) {
    this->desired_state_ = State::START_MICROPHONE;
  }
}

void MicTester::request_stop() {
  this->continuous_ = false;
  switch (this->state_) {
    case State::IDLE:
      break;
    case State::START_MICROPHONE:
    case State::STARTING_MICROPHONE:
    case State::DETECTING_SWEEP:
    case State::DETECTION_PAUSED:
      this->signal_stop_();
      this->set_state_(State::STOP_MICROPHONE, State::IDLE);
      break;
    case State::STOP_MICROPHONE:
    case State::STOPPING_MICROPHONE:
      this->desired_state_ = State::IDLE;
      break;
  }
}

void MicTester::pause_detection() {
  if (this->state_ == State::DETECTING_SWEEP) {
    this->continuous_ = false;
    this->set_state_(State::DETECTION_PAUSED);
  }
}

void MicTester::reset_detection() {
  this->clear_buffers_();
  mic_write_index = mic_filled = 0;
  std::fill_n(mic_buffer, MAX_BUFFER_SIZE, 0.0f);
  std::fill_n(work_buf, SWEEP_LEN, 0.0f);
  if (this->state_ == State::STARTING_MICROPHONE || this->state_ == State::DETECTING_SWEEP ||
      this->state_ == State::DETECTION_PAUSED) {
    this->continuous_ = true;
    this->set_state_(State::DETECTING_SWEEP);
  }
}

float MicTester::get_mic_energy() {
  if (this->energy_sample_count_ == 0)
    return -1.0f;
  const float rms = sqrtf(this->energy_accumulator_ / this->energy_sample_count_);
  this->energy_accumulator_ = 0.0f;
  this->energy_sample_count_ = 0;
  return rms;
}

void MicTester::set_udp_stream_enabled(bool enabled) {
  if (this->udp_stream_enabled_ == enabled)
    return;
  this->udp_stream_enabled_ = enabled;
  if (!enabled) {
    this->close_udp_socket_();
    this->reset_udp_packet_();
  }
}

void MicTester::set_udp_stream_host(const std::string &host) {
  this->udp_stream_host_ = host;
  this->udp_target_valid_ = false;
  this->close_udp_socket_();
}

void MicTester::set_udp_stream_port(uint16_t port) {
  this->udp_stream_port_ = port;
  this->udp_target_valid_ = false;
  this->close_udp_socket_();
}

void MicTester::set_udp_stream_packet_samples(size_t samples) {
  if (samples == 0)
    return;
  this->udp_stream_packet_samples_ = samples;
  this->udp_packet_buffer_.assign(samples, 0);
  this->udp_packet_fill_ = 0;
}

void MicTester::set_udp_stream_target(const std::string &host, uint16_t port) {
  this->udp_stream_host_ = host;
  this->udp_stream_port_ = port;
  this->udp_target_valid_ = false;
  this->close_udp_socket_();
}

bool MicTester::ensure_udp_socket_ready_() {
  if (!this->udp_stream_enabled_ || this->udp_stream_host_.empty() || this->udp_stream_port_ == 0)
    return false;
#ifdef USE_WIFI
  if (wifi::global_wifi_component == nullptr || !wifi::global_wifi_component->is_connected())
    return false;
#endif
  if (!this->udp_target_valid_) {
    struct in_addr address {};
    if (inet_aton(this->udp_stream_host_.c_str(), &address) == 0) {
      if (millis() - this->last_udp_error_log_ > 5000) {
        ESP_LOGW(TAG, "UDP target invalid: %s", this->udp_stream_host_.c_str());
        this->last_udp_error_log_ = millis();
      }
      return false;
    }
    std::memset(&this->udp_dest_addr_, 0, sizeof(this->udp_dest_addr_));
    this->udp_dest_addr_.sin_family = AF_INET;
    this->udp_dest_addr_.sin_port = htons(this->udp_stream_port_);
    this->udp_dest_addr_.sin_addr = address;
    this->udp_target_valid_ = true;
  }
  if (this->udp_socket_ < 0) {
    this->udp_socket_ = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->udp_socket_ < 0) {
      if (millis() - this->last_udp_error_log_ > 5000) {
        ESP_LOGW(TAG, "UDP socket open failed: errno %d", errno);
        this->last_udp_error_log_ = millis();
      }
      return false;
    }
  }
  return true;
}

void MicTester::close_udp_socket_() {
  if (this->udp_socket_ >= 0) {
    close(this->udp_socket_);
    this->udp_socket_ = -1;
  }
  this->udp_target_valid_ = false;
}

void MicTester::append_udp_sample_(int16_t sample) {
  if (!this->ensure_udp_socket_ready_() || this->udp_stream_packet_samples_ == 0)
    return;
  if (this->udp_packet_buffer_.size() != this->udp_stream_packet_samples_) {
    this->udp_packet_buffer_.assign(this->udp_stream_packet_samples_, 0);
    this->udp_packet_fill_ = 0;
  }
  this->udp_packet_buffer_[this->udp_packet_fill_++] = sample;
  if (this->udp_packet_fill_ < this->udp_stream_packet_samples_)
    return;
  const size_t bytes = this->udp_stream_packet_samples_ * sizeof(int16_t);
  if (lwip_sendto(this->udp_socket_, this->udp_packet_buffer_.data(), bytes, 0,
                  reinterpret_cast<struct sockaddr *>(&this->udp_dest_addr_), sizeof(this->udp_dest_addr_)) < 0 &&
      millis() - this->last_udp_error_log_ > 5000) {
    ESP_LOGW(TAG, "UDP send failed: errno %d", errno);
    this->last_udp_error_log_ = millis();
  }
  this->udp_packet_fill_ = 0;
}

void MicTester::reset_udp_packet_() { this->udp_packet_fill_ = 0; }

void MicTester::signal_stop_() {}

}  // namespace esphome::online_testing
