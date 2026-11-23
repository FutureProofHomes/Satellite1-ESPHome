// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "verifier_core.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <atomic>

namespace esphome::stt_wake_verifier {

struct Work {
  uint32_t id, started, timeout;
  std::string endpoint;
  RAMUniquePtr<uint8_t[]> pcm;
  size_t pcm_size{0};
};
struct WorkResult {
  uint32_t id, finished;
  std::string transcript, error;
};

class SttWakeVerifier : public Component, public microphone::Microphone {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void start() override;
  void stop() override;
  void begin(const std::string &wake_word);
  void cancel();
  void release_audio() { this->output_ready_ = true; }
  bool is_pending() const { return this->pending_; }
  void set_source(microphone::MicrophoneSource *source) { this->source_ = source; }
  void set_capture_allowed(std::function<bool()> allowed) { this->allowed_ = std::move(allowed); }
  void set_command_preroll(uint32_t ms) { this->command_preroll_ms_ = ms; }
  void set_enabled(bool enabled);
  void set_observe(bool observe) { this->observe_ = observe; }
  void set_endpoint(const std::string &value) { this->endpoint_ = value; }
  void set_threshold(float value) { this->threshold_ = clamp(value, 0.5f, 1.0f); }
  void set_window_ms(uint32_t value) { this->window_ms_ = clamp<uint32_t>(value, 500, 2000); }
  void set_timeout_ms(uint32_t value) { this->timeout_ms_ = clamp<uint32_t>(value, 500, 2000); }
  void set_result_sensor(text_sensor::TextSensor *value) { this->result_sensor_ = value; }
  void set_transcript_sensor(text_sensor::TextSensor *value) { this->transcript_sensor_ = value; }
  void set_score_sensor(sensor::Sensor *value) { this->score_sensor_ = value; }
  void set_duration_sensor(sensor::Sensor *value) { this->duration_sensor_ = value; }
  template<typename F> void add_verified_callback(F &&callback) { this->verified_.add(std::forward<F>(callback)); }
  template<typename F> void add_cancelled_callback(F &&callback) { this->cancelled_.add(std::forward<F>(callback)); }

 protected:
  static void worker_task_(void *context);
  static WorkResult *transcribe_(const Work &work);
  void on_audio_(const std::vector<uint8_t> &data);
  void allow_();
  void finish_(bool available, const std::string &transcript, const std::string &reason, bool late = false);
  void publish_(const std::string &result, const std::string &transcript, float score, uint32_t elapsed);
  microphone::MicrophoneSource *source_{nullptr};
  std::function<bool()> allowed_;
  SemaphoreHandle_t lock_{nullptr};
  QueueHandle_t work_queue_{nullptr}, result_queue_{nullptr};
  TaskHandle_t worker_{nullptr};
  RAMUniquePtr<int16_t[]> history_storage_, command_storage_;
  PcmBuffer history_, command_;
  std::vector<uint8_t> output_frame_;
  bool capturing_{false}, armed_{false}, output_ready_{false};
  std::atomic<bool> overflow_{false};
  bool enabled_{false}, observe_{false}, pending_{false}, check_observe_{false}, allowed_once_{false};
  bool worker_busy_{false}, timed_out_{false};
  bool check_enabled_{false}, buffering_available_{false};
  uint32_t generation_{0}, started_{0}, check_timeout_{500}, window_ms_{1000}, timeout_ms_{500};
  uint32_t command_preroll_ms_{250}, last_emit_{0}, armed_at_{0};
  float threshold_{0.85f}, check_threshold_{0.85f};
  std::string endpoint_, phrase_;
  CallbackManager<void(std::string)> verified_;
  CallbackManager<void(bool)> cancelled_;
  text_sensor::TextSensor *result_sensor_{nullptr}, *transcript_sensor_{nullptr};
  sensor::Sensor *score_sensor_{nullptr}, *duration_sensor_{nullptr};
};

class VerifiedTrigger : public Trigger<std::string> {
 public:
  explicit VerifiedTrigger(SttWakeVerifier *parent) {
    parent->add_verified_callback([this](const std::string &phrase) { this->trigger(phrase); });
  }
};

class CancelledTrigger : public Trigger<bool> {
 public:
  explicit CancelledTrigger(SttWakeVerifier *parent) {
    parent->add_cancelled_callback([this](bool recording) { this->trigger(recording); });
  }
};

}  // namespace esphome::stt_wake_verifier
