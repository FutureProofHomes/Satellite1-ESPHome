#include "wake_verify.h"

#include "esphome/components/json/json_util.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <esp_http_client.h>
#include <freertos/task.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iterator>

namespace esphome::wake_verify {

static const char *const TAG = "wake_verify";

static const uint32_t SAMPLE_RATE = 16000;
static const size_t MAX_RESPONSE_BYTES = 2048;
// esp_http_client and the JSON parse both run on this stack; the audio itself is in PSRAM.
static const uint32_t TASK_STACK_SIZE = 8192;
// Clips are boosted to this peak (-6 dBFS), by at most MAX_GAIN so near-silence stays quiet.
static const int32_t TARGET_PEAK = 16384;
static const float MAX_GAIN = 32.0f;

static float clip_gain(int32_t peak) {
  if (peak <= 0)
    return 1.0f;
  return std::clamp(TARGET_PEAK / (float) peak, 1.0f, MAX_GAIN);
}

void WakeVerify::setup() {
  // Off: no buffer and no audio callback; check() passes every wake straight through.
  if (this->mode_ == Mode::OFF)
    return;
  this->ring_samples_ = SAMPLE_RATE * this->buffer_duration_ms_ / 1000;
  RAMAllocator<int16_t> allocator;
  this->ring_ = allocator.allocate(this->ring_samples_);
  this->ring_lock_ = xSemaphoreCreateMutex();
  this->verdict_lock_ = xSemaphoreCreateMutex();
  if (this->ring_ == nullptr || this->ring_lock_ == nullptr || this->verdict_lock_ == nullptr) {
    ESP_LOGE(TAG, "Could not allocate the %u-sample audio buffer", (unsigned) this->ring_samples_);
    this->mark_failed();
    return;
  }
  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) { this->on_audio_(data); });
}

void WakeVerify::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Wake Verify:\n"
                "  Mode: %s\n"
                "  STT: %s/api/stt/%s (language %s)\n"
                "  Token: %s\n"
                "  Buffer: %u ms, post-roll %u ms, timeout %u ms, decision timeout %u ms\n"
                "  Match threshold: %.2f",
                this->mode_ == Mode::REROUTE ? "reroute" : this->mode_ == Mode::LOG ? "log only" : "off", this->ha_url_.c_str(), this->stt_engine_.c_str(),
                this->language_.c_str(), this->ha_token_.empty() ? "MISSING" : "set",
                (unsigned) this->buffer_duration_ms_, (unsigned) this->post_roll_ms_, (unsigned) this->timeout_ms_,
                (unsigned) this->decision_timeout_ms_, this->match_threshold_);
  for (const auto &word : this->skip_wake_words_)
    ESP_LOGCONFIG(TAG, "  Skips: '%s'", word.c_str());
}

void WakeVerify::on_audio_(const std::vector<uint8_t> &data) {
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  size_t count = data.size() / sizeof(int16_t);
  if (count == 0 || xSemaphoreTake(this->ring_lock_, pdMS_TO_TICKS(5)) != pdTRUE)
    return;
  while (count > 0) {
    size_t chunk = std::min(count, this->ring_samples_ - this->ring_write_);
    std::memcpy(this->ring_ + this->ring_write_, samples, chunk * sizeof(int16_t));
    samples += chunk;
    count -= chunk;
    this->ring_write_ += chunk;
    if (this->ring_write_ == this->ring_samples_) {
      this->ring_write_ = 0;
      this->ring_full_ = true;
    }
  }
  xSemaphoreGive(this->ring_lock_);
}

size_t WakeVerify::snapshot_(int16_t *out, size_t max_samples) {
  xSemaphoreTake(this->ring_lock_, portMAX_DELAY);
  size_t available = this->ring_full_ ? this->ring_samples_ : this->ring_write_;
  size_t count = std::min(available, max_samples);
  // Oldest of the newest `count` samples first.
  size_t start = (this->ring_write_ + this->ring_samples_ - count) % this->ring_samples_;
  size_t first = std::min(count, this->ring_samples_ - start);
  std::memcpy(out, this->ring_ + start, first * sizeof(int16_t));
  std::memcpy(out + first, this->ring_, (count - first) * sizeof(int16_t));
  xSemaphoreGive(this->ring_lock_);
  return count;
}

bool WakeVerify::is_skipped_(const std::string &wake_word) const {
  for (const auto &word : this->skip_wake_words_) {
    if (word == wake_word)
      return true;
  }
  return false;
}

void WakeVerify::check(const std::string &wake_word) {
  if (this->mode_ == Mode::OFF || this->is_failed() || this->is_skipped_(wake_word) || this->ha_token_.empty()) {
    this->wake_trigger_.trigger(wake_word);
    return;
  }
  if (this->waiting_) {
    // A second model firing on the same speech (two wake words that sound alike): the check under way hears the
    // same audio and already weighs every enabled wake word, so its verdict covers this one too.
    ESP_LOGI(TAG, "'%s' joins the check of '%s'", wake_word.c_str(), this->waiting_word_.c_str());
    return;
  }
  bool expected = false;
  if (!this->busy_.compare_exchange_strong(expected, true)) {
    ESP_LOGW(TAG, "'%s' not checked: the previous check is still running", wake_word.c_str());
    this->wake_trigger_.trigger(wake_word);
    return;
  }

  auto *job = new Job{++this->generation_, wake_word, {}};
  for (auto *model : this->micro_wake_word_->get_wake_words()) {
    if (model->is_enabled() && !this->is_skipped_(model->get_wake_word()))
      job->candidates.push_back(model->get_wake_word());
  }
  this->pending_job_ = job;
  if (xTaskCreate(WakeVerify::check_task, "wake_verify", TASK_STACK_SIZE, this, 2, nullptr) != pdPASS) {
    ESP_LOGW(TAG, "'%s' not checked: could not start the check task", wake_word.c_str());
    this->pending_job_ = nullptr;
    delete job;
    this->busy_ = false;
    this->wake_trigger_.trigger(wake_word);
    return;
  }

  if (this->mode_ == Mode::LOG) {
    this->wake_trigger_.trigger(wake_word);
  } else {
    this->waiting_ = true;
    this->waiting_since_ = millis();
    this->waiting_word_ = wake_word;
  }
}

void WakeVerify::loop() {
  if (!this->waiting_)
    return;
  std::string word;
  bool ready = false;
  xSemaphoreTake(this->verdict_lock_, portMAX_DELAY);
  if (this->verdict_ready_ && this->verdict_generation_ == this->generation_) {
    ready = true;
    word = this->verdict_word_;
    this->verdict_ready_ = false;
  }
  xSemaphoreGive(this->verdict_lock_);

  if (ready) {
    this->waiting_ = false;
    if (!word.empty())
      this->wake_trigger_.trigger(word);
  } else if (millis() - this->waiting_since_ > this->decision_timeout_ms_) {
    this->waiting_ = false;
    ESP_LOGW(TAG, "No verdict for '%s' within %u ms, waking unchecked", this->waiting_word_.c_str(),
             (unsigned) this->decision_timeout_ms_);
    this->wake_trigger_.trigger(this->waiting_word_);
  }
}

void WakeVerify::check_task(void *params) {
  auto *self = static_cast<WakeVerify *>(params);
  Job *job = self->pending_job_;
  self->pending_job_ = nullptr;
  self->run_check_(*job);
  delete job;
  self->busy_ = false;
  vTaskDelete(nullptr);
}

void WakeVerify::run_check_(Job &job) {
  vTaskDelay(pdMS_TO_TICKS(this->post_roll_ms_));

  std::string text;
  std::string reason;
  uint32_t elapsed = 0;
  int32_t peak = 0;
  float clip_s = 0.0f;
  // The verdict defaults to the detected word, so a failed check still wakes.
  std::string verdict = job.wake_word;
  const bool log_only = this->mode_ == Mode::LOG;

  if (!this->transcribe_(text, elapsed, peak, clip_s, reason)) {
    ESP_LOGW(TAG, "'%s' not checked, waking anyway: %s", job.wake_word.c_str(), reason.c_str());
  } else {
    float detected_score = match_score(text, job.wake_word);
    std::string best_word;
    float best_score = 0.0f;
    for (const auto &candidate : job.candidates) {
      float score = match_score(text, candidate);
      if (score > best_score) {
        best_score = score;
        best_word = candidate;
      }
    }
    const char *outcome;
    this->checks_++;
    if (detected_score >= this->match_threshold_) {
      outcome = "ACCEPT";
    } else if (best_score >= this->match_threshold_) {
      outcome = log_only ? "WOULD REROUTE" : "REROUTE";
      verdict = best_word;
      this->rerouted_++;
    } else {
      outcome = log_only ? "WOULD REJECT" : "REJECT";
      verdict.clear();
      this->rejected_++;
    }
    std::string arrow = !verdict.empty() && verdict != job.wake_word ? " -> '" + verdict + "'" : "";
    ESP_LOGI(TAG, "%s '%s'%s score=%.2f heard='%s' (%u ms, %.1fs clip, peak %d x%.1f) [%u checked: %u rerouted, %u rejected]",
             outcome, job.wake_word.c_str(), arrow.c_str(), detected_score, text.c_str(), (unsigned) elapsed, clip_s,
             (int) peak, clip_gain(peak), (unsigned) this->checks_,
             (unsigned) this->rerouted_, (unsigned) this->rejected_);
  }

  if (log_only)
    return;
  xSemaphoreTake(this->verdict_lock_, portMAX_DELAY);
  this->verdict_ready_ = true;
  this->verdict_generation_ = job.generation;
  this->verdict_word_ = verdict;
  xSemaphoreGive(this->verdict_lock_);
}

bool WakeVerify::transcribe_(std::string &text, uint32_t &elapsed_ms, int32_t &peak, float &clip_s,
                             std::string &reason) {
  RAMAllocator<int16_t> allocator;
  int16_t *audio = allocator.allocate(this->ring_samples_);
  if (audio == nullptr) {
    reason = "no memory for the clip";
    return false;
  }
  size_t samples = this->snapshot_(audio, this->ring_samples_);
  clip_s = samples / (float) SAMPLE_RATE;
  for (size_t i = 0; i < samples; i++)
    peak = std::max<int32_t>(peak, std::abs(static_cast<int32_t>(audio[i])));
  // Quiet clips (the Satellite1's peak around 200-500) go up to a normal level, which speech-to-text
  // hears better; a loud one is left alone.
  const float gain = clip_gain(peak);
  if (gain > 1.0f) {
    for (size_t i = 0; i < samples; i++)
      audio[i] = static_cast<int16_t>(audio[i] * gain);
  }

  std::string url = this->ha_url_ + "/api/stt/" + this->stt_engine_;
  std::string auth = "Bearer " + this->ha_token_;
  std::string content = "format=wav; codec=pcm; sample_rate=16000; bit_rate=16; channel=1; language=" + this->language_;

  esp_http_client_config_t config = {};
  config.url = url.c_str();
  config.method = HTTP_METHOD_POST;
  config.timeout_ms = static_cast<int>(this->timeout_ms_);
  config.disable_auto_redirect = true;
  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    allocator.deallocate(audio, this->ring_samples_);
    reason = "HTTP client init failed";
    return false;
  }
  esp_http_client_set_header(client, "Authorization", auth.c_str());
  esp_http_client_set_header(client, "X-Speech-Content", content.c_str());
  esp_http_client_set_header(client, "Content-Type", "application/octet-stream");

  const uint32_t started = millis();
  const int body_len = static_cast<int>(samples * sizeof(int16_t));
  std::string response;
  int status = -1;
  esp_err_t err = esp_http_client_open(client, body_len);
  if (err == ESP_OK) {
    const char *body = reinterpret_cast<const char *>(audio);
    int written = 0;
    while (written < body_len) {
      int n = esp_http_client_write(client, body + written, body_len - written);
      if (n <= 0)
        break;
      written += n;
    }
    if (written == body_len && esp_http_client_fetch_headers(client) >= 0) {
      status = esp_http_client_get_status_code(client);
      char buf[256];
      int n;
      while ((n = esp_http_client_read(client, buf, sizeof(buf))) > 0 && response.size() < MAX_RESPONSE_BYTES)
        response.append(buf, n);
    } else {
      err = ESP_FAIL;
    }
  }
  elapsed_ms = millis() - started;
  esp_http_client_cleanup(client);
  allocator.deallocate(audio, this->ring_samples_);

  if (err != ESP_OK || status != 200) {
    char buf[160];
    snprintf(buf, sizeof(buf), "STT failed (%s, HTTP %d, %u ms) %.80s", esp_err_to_name(err), status,
             (unsigned) elapsed_ms, response.c_str());
    reason = buf;
    return false;
  }
  json::parse_json(response, [&](JsonObject root) -> bool {
    text = root["text"] | "";
    return true;
  });
  return true;
}

// Second halves of "Nabu" that speech-to-text writes after "now": "Okay, now boo.", "Okay, now who?".
static const char *const NABU_TAILS[] = {"boo", "bu", "who", "wu", "woo", "lou", "poo", "vu", "voo"};

// Lowercase letters and digits, single spaces between words, "ok" spelled "okay", and "okay now boo" (and
// the other NABU_TAILS) read as "okay nabu". Other "okay, now ..." speech is left alone.
static std::vector<std::string> words_of(const std::string &s) {
  std::vector<std::string> words;
  std::string word;
  for (char c : s) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      word += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    } else if (c != '\'' && !word.empty()) {
      words.push_back(word);
      word.clear();
    }
  }
  if (!word.empty())
    words.push_back(word);
  for (auto &w : words) {
    if (w == "ok")
      w = "okay";
  }
  std::vector<std::string> joined;
  for (size_t i = 0; i < words.size(); i++) {
    if (i > 0 && i + 1 < words.size() && words[i - 1] == "okay" && words[i] == "now" &&
        std::find(std::begin(NABU_TAILS), std::end(NABU_TAILS), words[i + 1]) != std::end(NABU_TAILS)) {
      joined.push_back("nabu");
      i++;
      continue;
    }
    joined.push_back(words[i]);
  }
  return joined;
}

static float similarity(const std::string &a, const std::string &b) {
  if (a.empty() || b.empty())
    return 0.0f;
  std::vector<size_t> prev(b.size() + 1), cur(b.size() + 1);
  for (size_t j = 0; j <= b.size(); j++)
    prev[j] = j;
  for (size_t i = 1; i <= a.size(); i++) {
    cur[0] = i;
    for (size_t j = 1; j <= b.size(); j++) {
      size_t cost = a[i - 1] == b[j - 1] ? 0 : 1;
      cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + cost});
    }
    std::swap(prev, cur);
  }
  return 1.0f - prev[b.size()] / static_cast<float>(std::max(a.size(), b.size()));
}

float WakeVerify::match_score(const std::string &transcript, const std::string &phrase) {
  std::vector<std::string> heard = words_of(transcript);
  std::vector<std::string> target_words = words_of(phrase);
  std::string target;
  for (const auto &w : target_words)
    target += w;
  float best = 0.0f;
  // Compare without spaces so "Okay Na Bu" still matches "Okay Nabu"; allow one word more or fewer.
  const size_t n = target_words.size();
  for (size_t len = n > 1 ? n - 1 : 1; len <= n + 1; len++) {
    for (size_t start = 0; start + len <= heard.size(); start++) {
      std::string candidate;
      for (size_t k = start; k < start + len; k++)
        candidate += heard[k];
      best = std::max(best, similarity(candidate, target));
    }
  }
  return best;
}

}  // namespace esphome::wake_verify
