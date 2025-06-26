#include "udp_stream.h"

#include "esphome/core/log.h"

#include <cinttypes>
#include <cstdio>

namespace esphome {
namespace udp_stream {

static const char *const TAG = "udp_streamer";

#ifdef SAMPLE_RATE_HZ
#undef SAMPLE_RATE_HZ
#endif

static const size_t SAMPLE_RATE_HZ = 16000;
static const size_t INPUT_BUFFER_SIZE = 32 * SAMPLE_RATE_HZ / 1000;  // 32ms * 16kHz / 1000ms
static const size_t BUFFER_SIZE = 512 * SAMPLE_RATE_HZ / 1000;
static const size_t SEND_BUFFER_SIZE = INPUT_BUFFER_SIZE * sizeof(int16_t);
static const size_t RECEIVE_SIZE = 1024;
static const size_t SPEAKER_BUFFER_SIZE = 16 * RECEIVE_SIZE;

float UDPStreamer::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

bool UDPStreamer::start_udp_socket_() {
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  memset(&this->dest_addr_, 0, sizeof(this->dest_addr_));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(6055); // Port number in network byte order
  
  esp_ip_addr_t esp_ip = this->remote_ip_;
  server_addr.sin_addr.s_addr = esp_ip.u_addr.ip4.addr;
  
  memcpy(&this->dest_addr_, &server_addr, sizeof(server_addr));
  this->socket_ = socket::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (this->socket_ == nullptr) {
    ESP_LOGE(TAG, "Could not create socket");
    this->mark_failed();
    return false;
  }
  int enable = 1;
  int err = this->socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  if (err != 0) {
    ESP_LOGW(TAG, "Socket unable to set reuseaddr: errno %d", err);
    // we can still continue
  }
  err = this->socket_->setblocking(false);
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to set nonblocking mode: errno %d", err);
    this->mark_failed();
    return false;
  }

  this->udp_socket_running_ = true;
  return true;
}

void UDPStreamer::setup() {
  ESP_LOGCONFIG(TAG, "Setting up UDP Streamer...");
}

bool UDPStreamer::allocate_buffers_() {
  if (this->send_buffer_ != nullptr) {
    return true;  // Already allocated
  }

  ExternalRAMAllocator<int16_t> allocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
  this->input_buffer_ = allocator.allocate(INPUT_BUFFER_SIZE);
  if (this->input_buffer_ == nullptr) {
    ESP_LOGW(TAG, "Could not allocate input buffer");
    return false;
  }

  this->ring_buffer_ = RingBuffer::create(BUFFER_SIZE * sizeof(int16_t));
  if (this->ring_buffer_ == nullptr) {
    ESP_LOGW(TAG, "Could not allocate ring buffer");
    return false;
  }

  ExternalRAMAllocator<uint8_t> send_allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
  this->send_buffer_ = send_allocator.allocate(SEND_BUFFER_SIZE);
  if (send_buffer_ == nullptr) {
    ESP_LOGW(TAG, "Could not allocate send buffer");
    return false;
  }

  return true;
}

void UDPStreamer::clear_buffers_() {
  if (this->send_buffer_ != nullptr) {
    memset(this->send_buffer_, 0, SEND_BUFFER_SIZE);
  }

  if (this->input_buffer_ != nullptr) {
    memset(this->input_buffer_, 0, INPUT_BUFFER_SIZE * sizeof(int16_t));
  }

  if (this->ring_buffer_ != nullptr) {
    this->ring_buffer_->reset();
  }

}

void UDPStreamer::deallocate_buffers_() {
  ExternalRAMAllocator<uint8_t> send_deallocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
  send_deallocator.deallocate(this->send_buffer_, SEND_BUFFER_SIZE);
  this->send_buffer_ = nullptr;

  if (this->ring_buffer_ != nullptr) {
    this->ring_buffer_.reset();
    this->ring_buffer_ = nullptr;
  }


  ExternalRAMAllocator<int16_t> input_deallocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
  input_deallocator.deallocate(this->input_buffer_, INPUT_BUFFER_SIZE);
  this->input_buffer_ = nullptr;
}

int UDPStreamer::read_microphone_() {
  size_t bytes_read = 0;
  if (this->mic_->is_running()) {  // Read audio into input buffer
    bytes_read = this->mic_->read(this->input_buffer_, INPUT_BUFFER_SIZE * sizeof(int16_t), pdMS_TO_TICKS(10));
    if (bytes_read == 0) {
      memset(this->input_buffer_, 0, INPUT_BUFFER_SIZE * sizeof(int16_t));
      return 0;
    }
    // Write audio into ring buffer
    this->ring_buffer_->write((void *) this->input_buffer_, bytes_read);
  } else {
    ESP_LOGD(TAG, "microphone not running");
  }
  return bytes_read;
}

void UDPStreamer::loop() {
  switch (this->state_) {
    case State::IDLE: {
      if (this->continuous_ && this->desired_state_ == State::IDLE) {
        this->idle_trigger_->trigger();
        {
          this->set_state_(State::START_MICROPHONE, State::STREAMING_MICROPHONE);
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
    case State::STREAMING_MICROPHONE: {
      this->read_microphone_();
      size_t available = this->ring_buffer_->available();
      while (available >= SEND_BUFFER_SIZE) {
        size_t read_bytes = this->ring_buffer_->read((void *) this->send_buffer_, SEND_BUFFER_SIZE, 0);
        if (!this->udp_socket_running_) {
          if (!this->start_udp_socket_()) {
            this->set_state_(State::STOP_MICROPHONE, State::IDLE);
            break;
          }
        }
        this->socket_->sendto(this->send_buffer_, read_bytes, 0, (struct sockaddr *) &this->dest_addr_,
                                sizeof(this->dest_addr_));
        available = this->ring_buffer_->available();
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
    case State::STREAMING_MICROPHONE:
      return LOG_STR("STREAMING_MICROPHONE");
    case State::STOP_MICROPHONE:
      return LOG_STR("STOP_MICROPHONE");
    case State::STOPPING_MICROPHONE:
      return LOG_STR("STOPPING_MICROPHONE");
    default:
      return LOG_STR("UNKNOWN");
  }
};

void UDPStreamer::set_state_(State state) {
  State old_state = this->state_;
  this->state_ = state;
  ESP_LOGD(TAG, "State changed from %s to %s", LOG_STR_ARG(voice_assistant_state_to_string(old_state)),
           LOG_STR_ARG(voice_assistant_state_to_string(state)));
}

void UDPStreamer::set_state_(State state, State desired_state) {
  this->set_state_(state);
  this->desired_state_ = desired_state;
  ESP_LOGD(TAG, "Desired state set to %s", LOG_STR_ARG(voice_assistant_state_to_string(desired_state)));
}

void UDPStreamer::failed_to_start() {
  ESP_LOGE(TAG, "Failed to start server. See Home Assistant logs for more details.");
  this->error_trigger_->trigger("failed-to-start", "Failed to start server. See Home Assistant logs for more details.");
  this->set_state_(State::STOP_MICROPHONE, State::IDLE);
}


void UDPStreamer::request_start(bool continuous) {
  if (this->state_ == State::IDLE) {
    this->continuous_ = continuous;
    this->set_state_(State::START_MICROPHONE, State::STREAMING_MICROPHONE);
  }
}

void UDPStreamer::request_stop() {
  this->continuous_ = false;

  switch (this->state_) {
    case State::IDLE:
      break;
    case State::START_MICROPHONE:
    case State::STARTING_MICROPHONE:
      this->set_state_(State::STOP_MICROPHONE, State::IDLE);
      break;
    case State::STREAMING_MICROPHONE:
      this->signal_stop_();
      this->set_state_(State::STOP_MICROPHONE, State::IDLE);
      break;
    case State::STOP_MICROPHONE:
    case State::STOPPING_MICROPHONE:
      this->desired_state_ = State::IDLE;
      break;
  }
}

void UDPStreamer::signal_stop_() {
  memset(&this->dest_addr_, 0, sizeof(this->dest_addr_));
  this->udp_socket_running_ = false;
}


}  // namespace voice_assistant
}  // namespace esphome

