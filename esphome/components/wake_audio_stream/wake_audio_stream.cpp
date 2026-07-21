#include "wake_audio_stream.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

#include <cerrno>
#include <cstring>

namespace esphome {
namespace wake_audio_stream {

static const char *const TAG = "wake_audio_stream";

// microWakeWord feeds mono int16 PCM at 16 kHz.
static const size_t BYTES_PER_SECOND = 16000 * sizeof(int16_t);

// Max UDP payload we send per datagram. Kept below a typical MTU to avoid IP fragmentation. Must be a
// multiple of sizeof(int16_t) so we never split a sample across datagrams.
static const size_t SEND_CHUNK_SIZE = 1024;

float WakeAudioStream::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

void WakeAudioStream::setup() {
  size_t ring_bytes = BYTES_PER_SECOND * this->buffer_duration_ms_ / 1000;
  // Round up to a whole number of send chunks so draining is clean.
  ring_bytes = ((ring_bytes + SEND_CHUNK_SIZE - 1) / SEND_CHUNK_SIZE) * SEND_CHUNK_SIZE;
  this->ring_buffer_ = RingBuffer::create(ring_bytes);
  if (this->ring_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate %u byte ring buffer", (unsigned) ring_bytes);
    this->mark_failed();
    return;
  }
  this->send_buffer_.resize(SEND_CHUNK_SIZE);
}

void WakeAudioStream::dump_config() {
  ESP_LOGCONFIG(TAG, "Wake Audio Stream:");
  ESP_LOGCONFIG(TAG, "  Destination: %u.%u.%u.%u:%u", this->remote_ip_[0], this->remote_ip_[1], this->remote_ip_[2],
                this->remote_ip_[3], this->remote_port_);
  ESP_LOGCONFIG(TAG, "  Buffer duration: %u ms", (unsigned) this->buffer_duration_ms_);
  ESP_LOGCONFIG(TAG, "  Enabled on boot: %s", YESNO(this->enabled_));
}

bool WakeAudioStream::start_socket_() {
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  memset(&this->dest_addr_, 0, sizeof(this->dest_addr_));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(this->remote_port_);
  // s_addr is stored in network byte order, i.e. the octets laid out as [a, b, c, d] in memory.
  memcpy(&server_addr.sin_addr.s_addr, this->remote_ip_, sizeof(this->remote_ip_));

  memcpy(&this->dest_addr_, &server_addr, sizeof(server_addr));

  this->socket_ = socket::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (this->socket_ == nullptr) {
    ESP_LOGE(TAG, "Could not create socket");
    return false;
  }
  int err = this->socket_->setblocking(false);
  if (err != 0) {
    ESP_LOGE(TAG, "Could not set non-blocking mode: errno %d", errno);
    this->socket_ = nullptr;
    return false;
  }

  this->socket_running_ = true;
  return true;
}

void WakeAudioStream::stop_socket_() {
  if (this->socket_ != nullptr) {
    this->socket_->close();
    this->socket_ = nullptr;
  }
  this->socket_running_ = false;
}

void WakeAudioStream::set_enabled(bool enabled) {
  if (enabled == this->enabled_) {
    return;
  }
  this->enabled_ = enabled;
  if (enabled) {
    ESP_LOGI(TAG, "Wake-word audio capture enabled -> %u.%u.%u.%u:%u", this->remote_ip_[0], this->remote_ip_[1],
             this->remote_ip_[2], this->remote_ip_[3], this->remote_port_);
  } else {
    ESP_LOGI(TAG, "Wake-word audio capture disabled");
    this->stop_socket_();
    if (this->ring_buffer_ != nullptr) {
      this->ring_buffer_->reset();
    }
  }
}

void WakeAudioStream::push_audio(const std::vector<uint8_t> &data) {
  // Runs on the microphone task. Only buffer while enabled; the ring buffer is thread-safe and overwrites
  // the oldest data if the main loop cannot drain it fast enough.
  if (!this->enabled_ || this->ring_buffer_ == nullptr) {
    return;
  }
  this->ring_buffer_->write(data.data(), data.size());
}

void WakeAudioStream::loop() {
  if (!this->enabled_ || this->ring_buffer_ == nullptr) {
    return;
  }

  if (!this->socket_running_) {
    if (!this->start_socket_()) {
      // Back off; retry on a later loop.
      return;
    }
  }

  while (this->ring_buffer_->available() >= SEND_CHUNK_SIZE) {
    size_t read_bytes = this->ring_buffer_->read(this->send_buffer_.data(), SEND_CHUNK_SIZE, 0);
    if (read_bytes == 0) {
      break;
    }
    ssize_t sent = this->socket_->sendto(this->send_buffer_.data(), read_bytes, 0,
                                         (struct sockaddr *) &this->dest_addr_, sizeof(this->dest_addr_));
    if (sent < 0) {
      // ENOMEM/EAGAIN just means the stack is momentarily busy; drop this chunk and try again next loop.
      ESP_LOGV(TAG, "sendto failed: errno %d", errno);
      break;
    }
  }
}

}  // namespace wake_audio_stream
}  // namespace esphome

#endif  // USE_ESP32
