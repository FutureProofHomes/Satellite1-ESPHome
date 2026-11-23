// SPDX-License-Identifier: GPL-3.0-only
#include "stt_wake_verifier.h"

#include "wyoming_client.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cstring>
#include <memory>

namespace esphome::stt_wake_verifier {
static const char *const TAG = "stt_wake_verifier";
static constexpr size_t HISTORY_SAMPLES = 16000 * 2;
static constexpr size_t COMMAND_SAMPLES = 16000 * 6;
static constexpr size_t FRAME_SAMPLES = 512;

void SttWakeVerifier::setup() {
  this->lock_ = xSemaphoreCreateMutex();
  this->work_queue_ = xQueueCreate(1, sizeof(Work *));
  this->result_queue_ = xQueueCreate(1, sizeof(WorkResult *));
  RAMAllocator<int16_t> allocator(RAMAllocator<int16_t>::ALLOC_EXTERNAL);
  this->history_storage_ = allocator.make_unique_array_for_overwrite(HISTORY_SAMPLES);
  this->command_storage_ = allocator.make_unique_array_for_overwrite(COMMAND_SAMPLES);
  this->buffering_available_ = this->lock_ && this->history_storage_ && this->command_storage_;
  if (this->buffering_available_) {
    this->history_.bind(this->history_storage_.get(), HISTORY_SAMPLES);
    this->command_.bind(this->command_storage_.get(), COMMAND_SAMPLES);
  } else {
    ESP_LOGE(TAG, "Audio buffering unavailable; verification will allow listening");
  }
  this->source_->add_data_callback([this](const std::vector<uint8_t> &data) { this->on_audio_(data); });
  this->audio_stream_info_ = audio::AudioStreamInfo(16, 1, 16000);
  this->output_frame_.reserve(FRAME_SAMPLES * 2);
}

void SttWakeVerifier::dump_config() {
  ESP_LOGCONFIG(TAG, "Direct STT wake verification (Wyoming TCP)");
  ESP_LOGCONFIG(TAG, "  Command pre-roll: %lu ms", static_cast<unsigned long>(this->command_preroll_ms_));
  // The server address is intentionally not logged.
}

void SttWakeVerifier::set_enabled(bool enabled) {
  this->enabled_ = enabled;
  if (!enabled && this->pending_) {
    this->pending_ = false;
    ++this->generation_;
    this->check_enabled_ = false;
    this->publish_("Disabled during check — allowed", "", 0, millis() - this->started_);
    this->allow_();
  }
}

void SttWakeVerifier::on_audio_(const std::vector<uint8_t> &data) {
  if (!this->buffering_available_) {
    if (this->state_ == microphone::STATE_RUNNING && this->output_ready_)
      this->data_callbacks_.call(data);
    return;
  }
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  const auto *samples = reinterpret_cast<const int16_t *>(data.data());
  const size_t count = data.size() / sizeof(int16_t);
  this->history_.push(samples, count, true);
  if (this->armed_ || this->state_ == microphone::STATE_RUNNING) {
    if (!this->command_.push(samples, count))
      this->overflow_ = true;
  }
  xSemaphoreGive(this->lock_);
}

void SttWakeVerifier::start() {
  if (this->lock_)
    xSemaphoreTake(this->lock_, portMAX_DELAY);
  if (!this->armed_)
    this->command_.clear();
  this->state_ = microphone::STATE_RUNNING;
  this->output_ready_ = false;
  this->last_emit_ = millis();
  if (this->lock_)
    xSemaphoreGive(this->lock_);
}

void SttWakeVerifier::stop() {
  if (this->lock_)
    xSemaphoreTake(this->lock_, portMAX_DELAY);
  this->state_ = microphone::STATE_STOPPED;
  this->armed_ = this->output_ready_ = this->overflow_ = false;
  this->command_.clear();
  if (this->lock_)
    xSemaphoreGive(this->lock_);
}

void SttWakeVerifier::cancel() {
  const bool was_pending = this->pending_;
  this->pending_ = false;
  ++this->generation_;  // In-flight STT results cannot restart a cancelled run.
  this->stop();
  if (was_pending)
    this->publish_("Cancelled", "", 0, millis() - this->started_);
}

void SttWakeVerifier::allow_() {
  if (this->allowed_once_ || !this->allowed_ || !this->allowed_())
    return;
  this->allowed_once_ = true;
  this->verified_.call(this->phrase_);
}

void SttWakeVerifier::begin(const std::string &wake_word) {
  if (this->pending_ || this->armed_ || !this->allowed_ || !this->allowed_())
    return;
  this->phrase_ = wake_word.substr(0, 120);
  this->started_ = this->armed_at_ = millis();
  ++this->generation_;
  this->allowed_once_ = this->timed_out_ = false;
  this->check_observe_ = this->observe_;
  this->check_enabled_ = this->enabled_;
  this->check_threshold_ = this->threshold_;
  this->check_timeout_ = this->timeout_ms_;
  if (this->buffering_available_) {
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    this->command_.clear();
    // Seed directly into the empty command queue without another large allocation.
    const size_t count = this->history_.tail(this->command_storage_.get(), this->command_preroll_ms_ * 16);
    this->command_.adopt_prefix(count);
    this->armed_ = true;
    this->overflow_ = false;
    xSemaphoreGive(this->lock_);
  }
  if (!this->enabled_) {
    this->publish_("Disabled", "", 0, 0);
    this->allow_();
    return;
  }
  if (normalize_phrase(this->phrase_).empty()) {
    this->finish_(true, "", "Wake phrase unknown");
    return;
  }
  WyomingEndpoint endpoint;
  if (!parse_wyoming_endpoint(this->endpoint_, endpoint)) {
    this->finish_(false, "", "Wyoming server missing or invalid (use host:port)");
    return;
  }
  if (!this->buffering_available_ || !this->work_queue_ || !this->result_queue_) {
    this->finish_(false, "", "Audio buffering unavailable");
    return;
  }
  if (this->worker_busy_) {
    this->finish_(false, "", "Previous STT request still finishing");
    return;
  }
  if (!this->worker_ && xTaskCreate(worker_task_, "wake_stt", 8192, this, 2, &this->worker_) != pdPASS) {
    this->worker_ = nullptr;
    this->finish_(false, "", "STT worker unavailable");
    return;
  }
  auto work = std::make_unique<Work>();
  work->id = this->generation_;
  work->started = this->started_;
  work->timeout = this->check_timeout_;
  work->endpoint = this->endpoint_;
  auto allocator = RAMAllocator<uint8_t>(RAMAllocator<uint8_t>::ALLOC_EXTERNAL);
  work->pcm = allocator.make_unique_array_for_overwrite(this->window_ms_ * 32);
  if (!work->pcm) {
    this->finish_(false, "", "Wake snapshot unavailable");
    return;
  }
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  work->pcm_size = this->history_.tail(reinterpret_cast<int16_t *>(work->pcm.get()), this->window_ms_ * 16) * 2;
  xSemaphoreGive(this->lock_);
  if (work->pcm_size < 6400) {
    this->finish_(false, "", "Wake history not ready");
    return;
  }
  Work *queued = work.get();
  if (xQueueSend(this->work_queue_, &queued, 0) != pdTRUE) {
    this->finish_(false, "", "STT worker busy");
    return;
  }
  work.release();
  this->worker_busy_ = this->pending_ = true;
  this->publish_(this->check_observe_ ? "Observing" : "Checking", "", 0, 0);
  if (this->check_observe_)
    this->allow_();
}

void SttWakeVerifier::finish_(bool available, const std::string &transcript, const std::string &reason, bool late) {
  const float score = available ? transcript_match_score(transcript, this->phrase_) : 0;
  const bool matched = available && score >= this->check_threshold_;
  this->pending_ = false;
  const auto decision = evaluate_check(available, matched, this->check_observe_, late);
  const std::string result = decision.label ? decision.label : reason + " — allowed";
  this->publish_(result, transcript, score, millis() - this->started_);
  if (decision.allow)
    this->allow_();
  if (decision.reject)
    this->stop();
}

void SttWakeVerifier::publish_(const std::string &result, const std::string &transcript, float score,
                               uint32_t elapsed) {
  if (this->result_sensor_)
    this->result_sensor_->publish_state(result);
  if (this->transcript_sensor_)
    this->transcript_sensor_->publish_state(transcript.substr(0, 255));
  if (this->score_sensor_)
    this->score_sensor_->publish_state(score * 100);
  if (this->duration_sensor_)
    this->duration_sensor_->publish_state(elapsed);
  ESP_LOGD(TAG, "%s (%.1f%%, %lu ms)", result.c_str(), score * 100, static_cast<unsigned long>(elapsed));
}

void SttWakeVerifier::loop() {
  const bool allowed = this->allowed_ && this->allowed_();
  if (allowed != this->capturing_) {
    this->capturing_ = allowed;
    if (allowed)
      this->source_->start();
    else {
      const bool recording = this->state_ == microphone::STATE_RUNNING;
      this->cancel();
      this->cancelled_.call(recording);
      this->source_->stop();
      if (this->lock_)
        xSemaphoreTake(this->lock_, portMAX_DELAY);
      this->history_.clear();
      if (this->lock_)
        xSemaphoreGive(this->lock_);
    }
  }
  WorkResult *raw = nullptr;
  if (this->result_queue_ && xQueueReceive(this->result_queue_, &raw, 0) == pdTRUE) {
    this->worker_busy_ = false;
    std::unique_ptr<WorkResult> result(raw);
    if (result->id == this->generation_ && this->check_enabled_) {
      const bool late = this->timed_out_ || deadline_expired(result->finished, this->started_, this->check_timeout_);
      if (late && this->pending_) {
        this->timed_out_ = true;
        this->pending_ = false;
        this->allow_();
      }
      const bool valid =
          result->error.empty() && result->transcript.size() <= 255 && !normalize_phrase(result->transcript).empty();
      this->finish_(valid, result->transcript, result->error.empty() ? "STT returned no text" : result->error, late);
    }
  }
  if (this->pending_ && deadline_expired(millis(), this->started_, this->check_timeout_)) {
    this->pending_ = false;
    this->timed_out_ = true;
    this->publish_("Timeout — allowed", "", 0, millis() - this->started_);
    this->allow_();
  }
  if (!allowed || !this->buffering_available_)
    return;
  if (this->overflow_) {
    this->cancel();
    this->cancelled_.call(true);
    this->publish_("Command buffer full — cancelled", "", 0, millis() - this->started_);
    return;
  }
  // A failed/disconnected HA start must not leave a capture armed forever.
  if (this->armed_ && this->state_ == microphone::STATE_STOPPED && millis() - this->armed_at_ > 5000) {
    this->cancel();
    this->cancelled_.call(true);
  }
  // Wait until HA has actually started STT, then drain at most one 32 ms frame
  // every 16 ms. Never dump seconds of retained audio into VA's 512 ms ring.
  if (this->state_ == microphone::STATE_RUNNING && this->output_ready_ && millis() - this->last_emit_ >= 16) {
    auto &frame = this->output_frame_;
    frame.resize(FRAME_SAMPLES * 2);
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    const size_t count = this->command_.pop(reinterpret_cast<int16_t *>(frame.data()), FRAME_SAMPLES);
    xSemaphoreGive(this->lock_);
    if (count) {
      this->last_emit_ = millis();
      frame.resize(count * 2);
      this->data_callbacks_.call(frame);
    }
  }
}

WorkResult *SttWakeVerifier::transcribe_(const Work &work) {
  auto result = std::make_unique<WorkResult>();
  result->id = work.id;
  WyomingEndpoint endpoint;
  if (!parse_wyoming_endpoint(work.endpoint, endpoint)) {
    result->error = "Wyoming server missing or invalid (use host:port)";
    return result.release();
  }
  auto response = transcribe_wyoming(endpoint, work.pcm.get(), work.pcm_size, work.started, work.timeout,
                                     []() -> uint32_t { return millis(); });
  result->transcript = std::move(response.text);
  result->error = std::move(response.error);
  return result.release();
}

void SttWakeVerifier::worker_task_(void *context) {
  auto *self = static_cast<SttWakeVerifier *>(context);
  for (;;) {
    Work *raw = nullptr;
    if (xQueueReceive(self->work_queue_, &raw, portMAX_DELAY) != pdTRUE)
      continue;
    std::unique_ptr<Work> work(raw);
    WorkResult *result = transcribe_(*work);
    result->finished = millis();
    // Only the worker uses the Wyoming socket. Main-loop timeout/cancellation
    // never frees a live connection or PCM snapshot underneath it.
    if (xQueueSend(self->result_queue_, &result, portMAX_DELAY) != pdTRUE)
      delete result;
  }
}
}  // namespace esphome::stt_wake_verifier
