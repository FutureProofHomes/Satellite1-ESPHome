#pragma once

#include "esphome/components/micro_wake_word/micro_wake_word.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace esphome::wake_verify {

enum class Mode : uint8_t { OFF = 0, LOG = 1, REROUTE = 2 };

class WakeVerify : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_microphone_source(microphone::MicrophoneSource *source) { this->microphone_source_ = source; }
  void set_micro_wake_word(micro_wake_word::MicroWakeWord *mww) { this->micro_wake_word_ = mww; }
  void set_mode(uint8_t mode) { this->mode_ = static_cast<Mode>(mode); }
  void set_ha_url(const std::string &url) { this->ha_url_ = url; }
  void set_ha_token(const std::string &token) { this->ha_token_ = token; }
  void set_stt_engine(const std::string &engine) { this->stt_engine_ = engine; }
  void set_language(const std::string &language) { this->language_ = language; }
  void set_buffer_duration_ms(uint32_t ms) { this->buffer_duration_ms_ = ms; }
  void set_post_roll_ms(uint32_t ms) { this->post_roll_ms_ = ms; }
  void set_timeout_ms(uint32_t ms) { this->timeout_ms_ = ms; }
  void set_decision_timeout_ms(uint32_t ms) { this->decision_timeout_ms_ = ms; }
  void set_match_threshold(float threshold) { this->match_threshold_ = threshold; }
  void add_skip_wake_word(const std::string &word) { this->skip_wake_words_.push_back(word); }

  Trigger<std::string> *get_wake_trigger() { return &this->wake_trigger_; }

  /// Hand over a detected wake; on_wake fires with the word to use (now, or once the check is in).
  void check(const std::string &wake_word);

  /// Best fuzzy match (0-1) of phrase against any run of neighbouring words in transcript.
  static float match_score(const std::string &transcript, const std::string &phrase);

 protected:
  struct Job {
    uint32_t generation;
    std::string wake_word;
    std::vector<std::string> candidates;  // Enabled wake words, the detected one included.
  };

  static void check_task(void *params);
  void run_check_(Job &job);
  /// Transcribe the buffered audio. False if speech-to-text failed; `reason` says why.
  bool transcribe_(std::string &text, uint32_t &elapsed_ms, int32_t &peak, float &clip_s, std::string &reason);
  void on_audio_(const std::vector<uint8_t> &data);
  size_t snapshot_(int16_t *out, size_t max_samples);
  bool is_skipped_(const std::string &wake_word) const;

  microphone::MicrophoneSource *microphone_source_{nullptr};
  micro_wake_word::MicroWakeWord *micro_wake_word_{nullptr};
  Trigger<std::string> wake_trigger_;
  Mode mode_{Mode::LOG};
  std::string ha_url_;
  std::string ha_token_;
  std::string stt_engine_;
  std::string language_;
  uint32_t buffer_duration_ms_{1800};
  uint32_t post_roll_ms_{200};
  uint32_t timeout_ms_{3000};
  uint32_t decision_timeout_ms_{1500};
  float match_threshold_{0.75f};
  std::vector<std::string> skip_wake_words_;

  // Rolling buffer of the most recent mono 16 kHz samples, in PSRAM.
  int16_t *ring_{nullptr};
  size_t ring_samples_{0};
  size_t ring_write_{0};
  bool ring_full_{false};
  SemaphoreHandle_t ring_lock_{nullptr};

  std::atomic<bool> busy_{false};
  Job *pending_job_{nullptr};
  uint32_t generation_{0};

  // Reroute: the wake waiting in loop() for its verdict, and the verdict the task hands back.
  bool waiting_{false};
  uint32_t waiting_since_{0};
  std::string waiting_word_;
  SemaphoreHandle_t verdict_lock_{nullptr};
  bool verdict_ready_{false};
  uint32_t verdict_generation_{0};
  std::string verdict_word_;  // Empty: drop the wake.

  uint32_t checks_{0};
  uint32_t rerouted_{0};
  uint32_t rejected_{0};
};

template<typename... Ts> class CheckAction : public Action<Ts...>, public Parented<WakeVerify> {
  TEMPLATABLE_VALUE(std::string, wake_word)

 public:
  void play(const Ts &...x) override { this->parent_->check(this->wake_word_.value(x...)); }
};

}  // namespace esphome::wake_verify
