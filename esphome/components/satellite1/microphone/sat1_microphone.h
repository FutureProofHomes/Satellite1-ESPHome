#pragma once

#ifdef USE_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/core/component.h"
#include "esphome/core/ring_buffer.h"
#include "chunked_ring_buffer.h"

namespace esphome {
namespace nabu_microphone {

enum class TaskEventType : uint8_t {
  STARTING = 0,
  STARTED,
  RUNNING,
  IDLE,
  STOPPING,
  STOPPED,
  MUTED,
  WARNING = 255,
};

struct TaskEvent {
  TaskEventType type;
  esp_err_t err;
};

class NabuMicrophoneChannel;

class NabuMicrophone : public i2s_audio::I2SReader, public Component {
 public:
  void setup() override;
  void dump_config() override {this->dump_i2s_settings();}
  void start();
  void stop();

  void loop() override;

  void mute();
  void unmute();

  void set_channel_0(NabuMicrophoneChannel *microphone) { this->channel_0_ = microphone; }
  void set_channel_1(NabuMicrophoneChannel *microphone) { this->channel_1_ = microphone; }

  NabuMicrophoneChannel *get_channel_0() { return this->channel_0_; }
  NabuMicrophoneChannel *get_channel_1() { return this->channel_1_; }

  bool is_running() { return this->state_ == microphone::STATE_RUNNING; }
  uint32_t get_sample_rate() { return this->sample_rate_; }

 protected:
  esp_err_t start_i2s_driver_();

  microphone::State state_{microphone::STATE_STOPPED};

  static void read_task_(void *params);

  TaskHandle_t read_task_handle_{nullptr};
  QueueHandle_t event_queue_;

  NabuMicrophoneChannel *channel_0_{nullptr};
  NabuMicrophoneChannel *channel_1_{nullptr};
};

class NabuMicrophoneChannel : public microphone::Microphone, public Component {
 public:
  void setup() override;

  void start() override {
    this->parent_->start();
    this->is_muted_ = false;
    this->requested_stop_ = false;
  }

  void set_parent(NabuMicrophone *nabu_microphone) { this->parent_ = nabu_microphone; }

  void stop() override {
    this->requested_stop_ = true;
    this->is_muted_ = true;  // Mute until it is actually stopped
  };

  void loop() override;

  void set_mute_state(bool mute_state) override { this->is_muted_ = mute_state; }
  bool get_mute_state() { return this->is_muted_; }

  // void set_requested_stop() { this->requested_stop_ = true; }
  bool get_requested_stop() { return this->requested_stop_; }

  size_t read(int16_t *buf, size_t len, TickType_t ticks_to_wait = 0) override {
    return this->ring_buffer_->read((void *) buf, len, ticks_to_wait);
  };
  size_t read2(int16_t *buf, size_t len, TickType_t ticks_to_wait = 0) {
    return this->ring_buffer_->read((void *) buf, len, ticks_to_wait);
  };
  size_t read(int16_t *buf, size_t len) override { return this->ring_buffer_->read((void *) buf, len); };
  
  size_t read2(int16_t *buf, size_t len) { return this->ring_buffer2_->read((void *) buf, len); };
  
  void reset() override { this->ring_buffer_->reset(); }

  ChunkedRingBuffer *get_ring_buffer() { return this->ring_buffer_.get(); }
  RingBuffer *get_ring_buffer2() { return this->ring_buffer2_.get(); }

  void set_amplify_shift(uint8_t amplify_shift) { this->amplify_shift_ = amplify_shift; }
  uint8_t get_amplify_shift() { return this->amplify_shift_; }

 protected:
  NabuMicrophone *parent_;
  std::unique_ptr<ChunkedRingBuffer> ring_buffer_;
  std::unique_ptr<RingBuffer> ring_buffer2_;

  uint8_t amplify_shift_;
  bool is_muted_;
  bool requested_stop_;
};

}  // namespace nabu_microphone
}  // namespace esphome

#endif  // USE_ESP32
