#pragma once

#include "esphome/core/defines.h"

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/microphone/microphone.h"
#include "esphome/components/audio/audio.h"

#include <unordered_map>
#include <vector>

namespace esphome {
namespace online_testing {

enum class State {
  IDLE,
  START_MICROPHONE,
  STARTING_MICROPHONE,
  DETECTING_SWEEP,
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
  void set_media_file(audio::AudioFile *ref_media_file) { this->ref_media_file_ = ref_media_file; }

  void request_start(bool continuous);
  void request_stop();

  bool is_running() const { return this->state_ != State::IDLE; }
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
  int read_microphone_();
  void set_state_(State state);
  void set_state_(State state, State desired_state);
  void signal_stop_();

  Trigger<> *listening_trigger_ = new Trigger<>();
  Trigger<> *end_trigger_ = new Trigger<>();
  Trigger<> *start_trigger_ = new Trigger<>();
  Trigger<std::string, std::string> *error_trigger_ = new Trigger<std::string, std::string>();
  Trigger<> *idle_trigger_ = new Trigger<>();
  Trigger<> *sweep_detected_trigger_ = new Trigger<>();

  microphone::Microphone *mic_{nullptr};

  audio::AudioFile *ref_media_file_{nullptr};
  float sweep_norm_ = 0.0f;
  
  
  bool local_output_{false};

  HighFrequencyLoopRequester high_freq_;

  int16_t *input_buffer_;
  int read_pos_ = 0;

  bool continuous_{false};
  
  State state_{State::IDLE};
  State desired_state_{State::IDLE};
};


template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<MicTester> {
    TEMPLATABLE_VALUE(std::string, wake_word);
  
   public:
    void play(Ts... x) override {
      this->parent_->request_start(false);
    }
  };
  
  template<typename... Ts> class StartContinuousAction : public Action<Ts...>, public Parented<MicTester> {
   public:
    void play(Ts... x) override { this->parent_->request_start(true); }
  };
  
  template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<MicTester> {
   public:
    void play(Ts... x) override { this->parent_->request_stop(); }
  };
  
  template<typename... Ts> class IsRunningCondition : public Condition<Ts...>, public Parented<MicTester> {
   public:
    bool check(Ts... x) override { return this->parent_->is_running() || this->parent_->is_continuous(); }
  };
  







}  // namespace online_testing
}  // namespace esphome

