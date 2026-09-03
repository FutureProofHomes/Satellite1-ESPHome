#pragma once

#include "esphome/components/audio/audio.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <string>
#include <vector>

#ifdef USE_ESP32
extern "C" {
#include <lwip/sockets.h>
}
#endif

namespace esphome::online_testing {

enum class State {
  IDLE,
  START_MICROPHONE,
  STARTING_MICROPHONE,
  DETECTING_SWEEP,
  DETECTION_PAUSED,
  STOP_MICROPHONE,
  STOPPING_MICROPHONE,
};

class MicTester : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void failed_to_start();

  void set_microphone(microphone::Microphone *mic) { this->mic_ = mic; }
  void set_media_file(audio::AudioFile *media_file) { this->ref_media_file_ = media_file; }
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  uint8_t get_channel() const { return this->channel_; }

  void request_start(bool continuous);
  void request_stop();
  void pause_detection();
  void reset_detection();
  float get_mic_energy();

  void set_udp_stream_enabled(bool enabled);
  void set_udp_stream_host(const std::string &host);
  void set_udp_stream_port(uint16_t port);
  void set_udp_stream_packet_samples(size_t samples);
  void set_udp_stream_target(const std::string &host, uint16_t port);

  bool is_running() const { return this->state_ != State::IDLE; }
  bool is_mic_active() const {
    return this->state_ != State::IDLE && this->state_ != State::STOP_MICROPHONE &&
           this->state_ != State::STOPPING_MICROPHONE;
  }
  void set_continuous(bool continuous) { this->continuous_ = continuous; }
  bool is_continuous() const { return this->continuous_; }

  Trigger<> *get_end_trigger() const { return this->end_trigger_; }
  Trigger<> *get_start_trigger() const { return this->start_trigger_; }
  Trigger<std::string, std::string> *get_error_trigger() const { return this->error_trigger_; }
  Trigger<> *get_idle_trigger() const { return this->idle_trigger_; }
  Trigger<> *get_sweep_detected_trigger() const { return this->sweep_detected_trigger_; }

 protected:
  bool allocate_buffers_();
  void clear_buffers_();
  void deallocate_buffers_();
  void read_sweep_();
  void set_state_(State state);
  void set_state_(State state, State desired_state);
  void signal_stop_();
  bool ensure_udp_socket_ready_();
  void close_udp_socket_();
  void append_udp_sample_(int16_t sample);
  void reset_udp_packet_();
  void on_audio_data_(const std::vector<uint8_t> &data);

  Trigger<> *end_trigger_ = new Trigger<>();
  Trigger<> *start_trigger_ = new Trigger<>();
  Trigger<std::string, std::string> *error_trigger_ = new Trigger<std::string, std::string>();
  Trigger<> *idle_trigger_ = new Trigger<>();
  Trigger<> *sweep_detected_trigger_ = new Trigger<>();
  microphone::Microphone *mic_{nullptr};
  audio::AudioFile *ref_media_file_{nullptr};
  float sweep_norm_{0.0f};
  HighFrequencyLoopRequester high_freq_;
  int16_t *input_buffer_{nullptr};
  size_t input_buffer_size_{0};
  volatile size_t write_pos_{0};
  volatile size_t read_pos_{0};
  bool continuous_{false};
  bool callback_registered_{false};
  uint8_t channel_{0};
  float energy_accumulator_{0.0f};
  size_t energy_sample_count_{0};
  bool sweep_armed_{true};
  uint32_t last_sweep_ms_{0};
  bool udp_stream_enabled_{false};
  std::string udp_stream_host_{};
  uint16_t udp_stream_port_{0};
  size_t udp_stream_packet_samples_{960};
  std::vector<int16_t> udp_packet_buffer_{};
  size_t udp_packet_fill_{0};
  int udp_socket_{-1};
  bool udp_target_valid_{false};
  struct sockaddr_in udp_dest_addr_ {};
  uint32_t last_udp_error_log_{0};
  State state_{State::IDLE};
  State desired_state_{State::IDLE};
};

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<MicTester> {
 public:
  void play(const Ts &...x) override { this->parent_->request_start(false); }
};

template<typename... Ts> class StartContinuousAction : public Action<Ts...>, public Parented<MicTester> {
 public:
  void play(const Ts &...x) override { this->parent_->request_start(true); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<MicTester> {
 public:
  void play(const Ts &...x) override { this->parent_->request_stop(); }
};

}  // namespace esphome::online_testing
