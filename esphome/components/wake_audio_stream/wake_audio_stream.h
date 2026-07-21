#pragma once

#ifdef USE_ESP32

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/ring_buffer.h"

#include "esphome/components/socket/socket.h"

#include <memory>
#include <vector>

namespace esphome {
namespace wake_audio_stream {

/// Streams raw audio (int16 PCM, mono, 16 kHz) off-device over UDP so it can be recorded and labeled as
/// wake-word training data. It is fed by microWakeWord's `on_audio_data` trigger, which delivers the exact
/// audio the wake-word models process (so both detections and misses can be captured). Audio arrives on the
/// microphone task; it is buffered in a thread-safe ring buffer and flushed to the socket from the main loop.
class WakeAudioStream : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_remote_ip(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    this->remote_ip_[0] = a;
    this->remote_ip_[1] = b;
    this->remote_ip_[2] = c;
    this->remote_ip_[3] = d;
  }
  void set_remote_port(uint16_t port) { this->remote_port_ = port; }
  void set_buffer_duration_ms(uint32_t duration_ms) { this->buffer_duration_ms_ = duration_ms; }

  /// Enable or disable capture. Toggled from a switch/action.
  void set_enabled(bool enabled);
  bool is_running() const { return this->enabled_; }

  /// Push a chunk of raw audio into the send buffer. Called from the microphone task via the trigger.
  void push_audio(const std::vector<uint8_t> &data);

 protected:
  bool start_socket_();
  void stop_socket_();

  uint8_t remote_ip_[4]{0, 0, 0, 0};
  uint16_t remote_port_{6056};
  uint32_t buffer_duration_ms_{500};

  bool enabled_{false};

  std::unique_ptr<RingBuffer> ring_buffer_;
  std::vector<uint8_t> send_buffer_;

  std::unique_ptr<socket::Socket> socket_{nullptr};
  struct sockaddr_storage dest_addr_;
  bool socket_running_{false};
};

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<WakeAudioStream> {
 public:
  void play(Ts... x) override { this->parent_->set_enabled(true); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<WakeAudioStream> {
 public:
  void play(Ts... x) override { this->parent_->set_enabled(false); }
};

template<typename... Ts> class IsRunningCondition : public Condition<Ts...>, public Parented<WakeAudioStream> {
 public:
  bool check(Ts... x) override { return this->parent_->is_running(); }
};

}  // namespace wake_audio_stream
}  // namespace esphome

#endif  // USE_ESP32
