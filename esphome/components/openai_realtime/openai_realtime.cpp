#include "openai_realtime.h"

#ifdef USE_ESP32

#include <cctype>
#include <cinttypes>
#include <cstring>

#include <esp_heap_caps.h>
#include <esp_http_client.h>
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include <esp_crt_bundle.h>
#endif

#include "esphome/components/audio/audio.h"
#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace openai_realtime {

static const char *const TAG = "openai_realtime";

// 24 kHz, 16-bit, mono: 48 bytes per millisecond.
static constexpr uint32_t BYTES_PER_MS = 48;
// One input_audio_buffer.append per 60 ms of microphone audio. 2880 bytes of PCM become 3840 base64
// characters, so every append is well inside the client buffer and leaves as one frame.
static constexpr size_t MIC_CHUNK = 60 * BYTES_PER_MS;
static constexpr size_t MIC_RING_SIZE = 256 * 1024;  // ~5 s: covers the connect, so the first words are kept
static constexpr size_t RX_BUF_SIZE = 512 * 1024;    // largest server event we accept (audio deltas)
static constexpr size_t TX_BUF_SIZE = 8 * 1024;
static constexpr size_t PLAY_CHUNK = 4096;
static constexpr uint32_t CONNECT_TIMEOUT_MS = 20000;
// After session.update, a server that never confirms with session.updated (several self-hosted ones)
// is taken as configured after this long without an error.
static constexpr uint32_t CONFIGURE_GRACE_MS = 3000;
// How long to wait after the WebSocket opens for session.created, which tells GA from beta.
static constexpr uint32_t SESSION_CREATED_WAIT_MS = 2500;
static constexpr size_t MODELS_BODY_MAX = 384 * 1024;
static constexpr size_t VOICES_BODY_MAX = 64 * 1024;
static constexpr const char *OPENAI_ORIGIN = "https://api.openai.com";

static void *psram_alloc(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (p == nullptr)
    p = heap_caps_malloc(n, MALLOC_CAP_8BIT);
  return p;
}

const char *session_state_name(SessionState s) {
  switch (s) {
    case SessionState::CONNECTING:
      return "connecting";
    case SessionState::ACTIVE:
      return "active";
    case SessionState::CLOSING:
      return "closing";
    default:
      return "idle";
  }
}

const char *OpenAIRealtime::get_flavor_name() const {
  switch (static_cast<Flavor>(this->flavor_.load())) {
    case Flavor::GA:
      return "ga";
    case Flavor::BETA:
      return "beta";
    default:
      return "";
  }
}

const char *phase_name(Phase p) {
  switch (p) {
    case Phase::LISTENING:
      return "listening";
    case Phase::USER_SPEAKING:
      return "user_speaking";
    case Phase::THINKING:
      return "thinking";
    case Phase::SPEAKING:
      return "speaking";
    default:
      return "none";
  }
}

// =================================================================================================
// Setup, settings
// =================================================================================================

void OpenAIRealtime::setup() {
  this->load_settings_();
  if (this->mic_source_ != nullptr) {
    this->mic_source_->add_data_callback([this](const std::vector<uint8_t> &data) { this->on_mic_data_(data); });
  }
  this->up_buf_.reserve(4096);
}

void OpenAIRealtime::dump_config() {
  const Settings s = this->get_settings();
  Endpoints ep;
  std::string err;
  const bool ok = derive_endpoints(s.base_url, s.model, ep, err);
  ESP_LOGCONFIG(TAG,
                "OpenAI Realtime:\n"
                "  Enabled: %s\n"
                "  Base URL: %s\n"
                "  Realtime URL: %s\n"
                "  Model: %s\n"
                "  Voice: %s\n"
                "  API key: %s\n"
                "  Turn detection: %s\n"
                "  Half duplex: %s\n"
                "  Idle timeout: %" PRIu32 " ms",
                YESNO(s.enabled), s.base_url.c_str(), ok ? ep.realtime_url.c_str() : "(invalid)", s.model.c_str(),
                s.voice.c_str(), s.api_key.empty() ? "not set" : "set", this->td_type_.c_str(),
                YESNO(this->half_duplex_), this->idle_timeout_ms_);
}

void OpenAIRealtime::load_settings_() {
  LockGuard guard(this->settings_lock_);
  this->settings_ = this->default_;
  this->pref_ = global_preferences->make_preference<SettingsBlob>(fnv1_hash("openai_realtime_settings_v2"));
  auto *blob = new SettingsBlob();  // ~550 bytes; off the loop task's stack
  bool loaded = this->pref_.load(blob) && blob->magic == SETTINGS_MAGIC;
  if (!loaded) {
    // Settings saved by the first release (23-character voice field): read them once and rewrite
    // them in the current layout.
    auto pref_v1 = global_preferences->make_preference<SettingsBlobV1>(fnv1_hash("openai_realtime_settings_v1"));
    auto *old = new SettingsBlobV1();
    if (pref_v1.load(old) && old->magic == SETTINGS_MAGIC_V1) {
      memset(blob, 0, sizeof(*blob));
      blob->magic = SETTINGS_MAGIC;
      blob->enabled = old->enabled;
      memcpy(blob->base_url, old->base_url, sizeof(old->base_url));
      memcpy(blob->model, old->model, sizeof(old->model));
      memcpy(blob->voice, old->voice, sizeof(old->voice));
      memcpy(blob->api_key, old->api_key, sizeof(old->api_key));
      blob->voice[sizeof(old->voice) - 1] = '\0';
      this->pref_.save(blob);
      loaded = true;
      ESP_LOGI(TAG, "Migrated the stored settings to the current layout");
    }
    memset(old, 0, sizeof(*old));
    delete old;
  }
  if (loaded) {
    blob->base_url[MAX_BASE_URL] = '\0';
    blob->model[MAX_MODEL] = '\0';
    blob->voice[MAX_VOICE] = '\0';
    blob->api_key[MAX_API_KEY] = '\0';
    this->settings_.enabled = blob->enabled != 0;
    if (blob->base_url[0])
      this->settings_.base_url = blob->base_url;
    if (blob->model[0])
      this->settings_.model = blob->model;
    if (blob->voice[0])
      this->settings_.voice = blob->voice;
    // An empty stored key falls back to a YAML key, so a fleet can ship one in secrets.yaml.
    if (blob->api_key[0])
      this->settings_.api_key = blob->api_key;
  }
  memset(blob, 0, sizeof(*blob));
  delete blob;
}

void OpenAIRealtime::save_settings_locked_() {
  auto *blob = new SettingsBlob();
  memset(blob, 0, sizeof(*blob));
  blob->magic = SETTINGS_MAGIC;
  blob->enabled = this->settings_.enabled ? 1 : 0;
  strncpy(blob->base_url, this->settings_.base_url.c_str(), MAX_BASE_URL);
  strncpy(blob->model, this->settings_.model.c_str(), MAX_MODEL);
  strncpy(blob->voice, this->settings_.voice.c_str(), MAX_VOICE);
  strncpy(blob->api_key, this->settings_.api_key.c_str(), MAX_API_KEY);
  // No sync() here: this runs on the web server's task, and ESPHome flushes pending preference
  // writes from its own loop - the same discipline the Music Assistant settings follow.
  this->pref_.save(blob);
  memset(blob, 0, sizeof(*blob));
  delete blob;
}

Settings OpenAIRealtime::get_settings() const {
  LockGuard guard(this->settings_lock_);
  return this->settings_;
}

bool OpenAIRealtime::is_configured() const {
  const Settings s = this->get_settings();
  Endpoints ep;
  std::string err;
  if (!derive_endpoints(s.base_url, s.model, ep, err) || s.model.empty())
    return false;
  // A relay may hold the key itself; api.openai.com never accepts an unauthenticated session.
  return !s.api_key.empty() || ep.origin != OPENAI_ORIGIN;
}

bool OpenAIRealtime::is_enabled() const { return this->get_settings().enabled && this->is_configured(); }

static bool valid_token(const std::string &s, size_t max, const char *extra) {
  if (s.empty() || s.size() > max)
    return false;
  for (unsigned char c : s) {
    if (c != 0 && (std::isalnum(c) || strchr(extra, c) != nullptr))
      continue;
    return false;
  }
  return true;
}

bool OpenAIRealtime::apply_settings(bool enabled, const std::string &base_url, const std::string &model,
                                    const std::string &voice, const std::string *api_key, std::string &error) {
  const std::string base = normalize_base_url(base_url);
  Endpoints ep;
  std::string why;
  if (base.size() > MAX_BASE_URL || !derive_endpoints(base, model, ep, why)) {
    error = "base_url";
    return false;
  }
  if (!valid_token(model, MAX_MODEL, "-_.:/")) {
    error = "model";
    return false;
  }
  if (!valid_token(voice, MAX_VOICE, "-_.:/+")) {
    error = "voice";
    return false;
  }
  if (api_key != nullptr && !api_key->empty()) {
    if (api_key->size() > MAX_API_KEY) {
      error = "api_key";
      return false;
    }
    for (unsigned char c : *api_key) {
      if (c <= 0x20 || c >= 0x7f || c == '"' || c == '\\') {
        error = "api_key";
        return false;
      }
    }
  }

  LockGuard guard(this->settings_lock_);
  Settings next = this->settings_;
  // A key saved for one host is never carried to another: changing the origin without typing a key
  // drops the stored one, so the model list or a session can't hand it to the new server.
  const bool origin_changed = url_origin(next.base_url) != ep.origin;
  next.enabled = enabled;
  next.base_url = base;
  next.model = model;
  next.voice = voice;
  if (api_key != nullptr)
    next.api_key = *api_key;
  else if (origin_changed)
    next.api_key.clear();
  const bool changed = next.enabled != this->settings_.enabled || next.base_url != this->settings_.base_url ||
                       next.model != this->settings_.model || next.voice != this->settings_.voice ||
                       next.api_key != this->settings_.api_key;
  if (changed) {
    this->settings_ = next;
    this->save_settings_locked_();
    ESP_LOGI(TAG, "Settings saved: %s, model %s, voice %s, key %s", this->settings_.base_url.c_str(),
             this->settings_.model.c_str(), this->settings_.voice.c_str(),
             this->settings_.api_key.empty() ? "not set" : "set");
  }
  return true;
}

std::string OpenAIRealtime::get_last_error() const {
  LockGuard guard(this->status_lock_);
  return this->last_error_;
}

TestStatus OpenAIRealtime::get_test_status() const {
  LockGuard guard(this->status_lock_);
  return this->test_;
}

uint32_t OpenAIRealtime::request_test() {
  uint32_t gen;
  {
    LockGuard guard(this->status_lock_);
    gen = ++this->test_.gen;
    this->test_.state = 1;
    this->test_.message.clear();
  }
  this->test_requested_.store(true);
  App.wake_loop_threadsafe();
  return gen;
}

static void set_test_result(Mutex &lock, TestStatus &t, bool ok, const std::string &msg) {
  LockGuard guard(lock);
  if (t.state != 1)
    return;
  t.state = ok ? 2 : 3;
  t.message = msg;
}

// =================================================================================================
// Session lifecycle (main loop)
// =================================================================================================

void OpenAIRealtime::request_start() {
  if (this->is_active()) {
    ESP_LOGD(TAG, "Start ignored: a session is already running");
    return;
  }
  if (this->muted_fn_ && this->muted_fn_()) {
    ESP_LOGI(TAG, "Start ignored: microphones are muted");
    return;
  }
  if (!this->is_configured()) {
    this->fail_("not_configured", "OpenAI Realtime is not configured - set it up under Settings > OpenAI");
    return;
  }
  this->start_session_(false);
}

void OpenAIRealtime::request_stop() {
  const SessionState s = this->state_.load();
  if (s == SessionState::IDLE || s == SessionState::CLOSING)
    return;
  ESP_LOGD(TAG, "Stopping session");
  this->stop_requested_ = true;
  this->state_.store(SessionState::CLOSING);
  this->worker_stop_.store(true);
  this->accepting_mic_.store(false);
  if (!this->test_only_) {
    if (this->mic_source_ != nullptr)
      this->mic_source_->stop();
    if (this->speaker_ != nullptr)
      this->speaker_->stop();
    this->playing_audio_.store(false);
  }
}

void OpenAIRealtime::fail_(const std::string &code, const std::string &message) {
  ESP_LOGW(TAG, "%s: %s", code.c_str(), message.c_str());
  {
    LockGuard guard(this->status_lock_);
    this->last_error_ = message;
  }
  if (this->test_only_) {
    set_test_result(this->status_lock_, this->test_, false, message);
  } else {
    this->error_trigger_.trigger(code, message);
  }
  this->request_stop();
}

void OpenAIRealtime::build_session_update_(const Settings &s) {
  // Both shapes are built up front; the worker sends the one the server's session.created asks for.
  SessionConfig c;
  c.model = s.model;
  c.voice = s.voice;
  c.instructions = this->instructions_;
  c.td_type = this->td_type_;
  c.td_eagerness = this->td_eagerness_;
  c.td_threshold = this->td_threshold_;
  c.td_prefix_ms = this->td_prefix_ms_;
  c.td_silence_ms = this->td_silence_ms_;
  c.noise_reduction = this->noise_reduction_;
  c.transcription_model = this->transcription_model_;
  c.tools_json = this->tools_json_;
  c.end_tool = this->end_tool_;
  this->session_update_ = json::build_json([&c](JsonObject root) { fill_session_update(root, c, Flavor::GA); });
  this->session_update_beta_ =
      json::build_json([&c](JsonObject root) { fill_session_update(root, c, Flavor::BETA); });
}

void OpenAIRealtime::start_session_(bool test_only) {
  const Settings s = this->get_settings();
  Endpoints ep;
  std::string why;
  if (!derive_endpoints(s.base_url, s.model, ep, why)) {
    this->test_only_ = test_only;
    this->fail_("bad_url", "The base URL is not a valid http(s):// or ws(s):// address");
    this->test_only_ = false;
    return;
  }

  // Buffers live only for the session, except the microphone ring: the microphone task may still be
  // inside its callback when a session ends, so that one is allocated once and only reset.
  if (this->rx_buf_ == nullptr) {
    this->rx_buf_ = static_cast<char *>(psram_alloc(RX_BUF_SIZE));
    this->rx_cap_ = this->rx_buf_ == nullptr ? 0 : RX_BUF_SIZE;
  }
  if (this->tx_buf_ == nullptr) {
    this->tx_buf_ = static_cast<char *>(psram_alloc(TX_BUF_SIZE));
    this->tx_cap_ = this->tx_buf_ == nullptr ? 0 : TX_BUF_SIZE;
  }
  if (this->pcm_buf_ == nullptr)
    this->pcm_buf_ = static_cast<uint8_t *>(psram_alloc(MIC_CHUNK));
  if (this->play_buf_ == nullptr)
    this->play_buf_ = static_cast<uint8_t *>(psram_alloc(PLAY_CHUNK));
  if (!test_only) {
    if (this->mic_ring_ == nullptr)
      this->mic_ring_ = ring_buffer::RingBuffer::create(MIC_RING_SIZE);
    if (this->playback_ring_ == nullptr)
      this->playback_ring_ = ring_buffer::RingBuffer::create(this->playback_buffer_size_);
  }
  if (this->rx_buf_ == nullptr || this->tx_buf_ == nullptr || this->pcm_buf_ == nullptr ||
      this->play_buf_ == nullptr || (!test_only && (this->mic_ring_ == nullptr || this->playback_ring_ == nullptr))) {
    this->test_only_ = test_only;
    this->release_buffers_();
    this->fail_("no_memory", "Not enough memory to start a conversation");
    this->test_only_ = false;
    return;
  }

  this->test_only_ = test_only;
  this->ws_url_ = ep.realtime_url;
  this->auth_header_ = s.api_key.empty() ? "" : "Bearer " + s.api_key;
  this->build_session_update_(s);

  // Session flags.
  this->stop_requested_ = false;
  this->response_active_ = false;
  this->need_response_create_ = false;
  this->pending_calls_ = 0;
  this->end_after_playback_ = false;
  this->ready_fired_ = false;
  this->phase_ = Phase::NONE;
  this->session_started_ms_ = millis();
  this->last_activity_ms_ = this->session_started_ms_;
  this->bytes_written_.store(0);
  this->bytes_played_ = 0;
  this->current_item_offset_.store(0);
  this->first_play_offset_ = UINT32_MAX;
  this->item_first_play_ms_ = 0;
  this->play_len_ = this->play_off_ = 0;
  this->barge_in_pending_.store(false);
  this->playing_audio_.store(false);
  this->ws_response_active_.store(false);
  {
    LockGuard guard(this->item_lock_);
    this->current_item_.clear();
    this->dropped_item_.clear();
    this->barge_item_.clear();
    this->barge_offset_ = 0;
  }
  {
    LockGuard guard(this->out_lock_);
    this->out_queue_.clear();
  }
  {
    LockGuard guard(this->event_lock_);
    this->events_.clear();
  }
  this->rx_len_ = 0;
  this->rx_overflow_ = false;
  this->handshake_status_ = 0;
  this->close_reason_.clear();
  this->worker_stop_.store(false);
  this->worker_done_.store(false);
  this->ws_connected_.store(false);
  this->session_configured_.store(false);
  this->session_created_.store(false);
  this->flavor_.store(static_cast<uint8_t>(Flavor::UNKNOWN));
  this->transport_failed_.store(false);

  if (!test_only) {
    this->mic_ring_->reset();
    this->playback_ring_->reset();
    this->upsampler_.reset();
    this->speaker_->stop();
    this->speaker_->set_audio_stream_info(audio::AudioStreamInfo(16, 1, 24000));
    this->accepting_mic_.store(true);
    this->mic_source_->start();
  }

  this->state_.store(SessionState::CONNECTING);
  // The worker owns the socket for the session's life. Internal stack: it runs mbedTLS writes.
  if (xTaskCreate(&OpenAIRealtime::worker_task_, "oai_rt", 8192, this, 5, &this->worker_handle_) != pdPASS) {
    this->worker_handle_ = nullptr;
    this->state_.store(SessionState::IDLE);
    this->accepting_mic_.store(false);
    if (!test_only)
      this->mic_source_->stop();
    this->fail_("no_memory", "Could not start the connection task");
    this->test_only_ = false;
    return;
  }
  this->high_freq_.start();
  ESP_LOGI(TAG, "%s %s", test_only ? "Testing" : "Connecting to", this->ws_url_.c_str());
  if (!test_only)
    this->start_trigger_.trigger();
}

void OpenAIRealtime::release_buffers_() {
  // Everything but the microphone ring (see start_session_). Only called with no worker running,
  // so neither the WebSocket task nor the worker can be touching these.
  this->playback_ring_.reset();
  heap_caps_free(this->rx_buf_);
  this->rx_buf_ = nullptr;
  this->rx_cap_ = 0;
  heap_caps_free(this->tx_buf_);
  this->tx_buf_ = nullptr;
  this->tx_cap_ = 0;
  heap_caps_free(this->pcm_buf_);
  this->pcm_buf_ = nullptr;
  heap_caps_free(this->play_buf_);
  this->play_buf_ = nullptr;
}

void OpenAIRealtime::finish_session_() {
  this->worker_handle_ = nullptr;
  const bool was_test = this->test_only_;
  const bool unexpected = !this->stop_requested_;
  this->accepting_mic_.store(false);
  if (!was_test) {
    if (this->mic_source_ != nullptr)
      this->mic_source_->stop();
    if (this->speaker_ != nullptr)
      this->speaker_->stop();
  }
  this->playing_audio_.store(false);
  this->release_buffers_();
  this->high_freq_.stop();
  this->phase_ = Phase::NONE;
  this->state_.store(SessionState::IDLE);

  if (unexpected) {
    std::string msg = this->close_reason_.empty() ? "The server closed the connection" : this->close_reason_;
    if (was_test) {
      set_test_result(this->status_lock_, this->test_, false, msg);
    } else {
      ESP_LOGW(TAG, "%s", msg.c_str());
      {
        LockGuard guard(this->status_lock_);
        this->last_error_ = msg;
      }
      this->error_trigger_.trigger("disconnected", msg);
    }
  }
  if (was_test) {
    set_test_result(this->status_lock_, this->test_, false, "The test ended without an answer");
  } else {
    this->end_trigger_.trigger();
  }
  this->test_only_ = false;
  ESP_LOGI(TAG, "Session ended");
}

void OpenAIRealtime::set_phase_(Phase p) {
  if (this->phase_ == p)
    return;
  this->phase_ = p;
  if (this->test_only_)
    return;
  switch (p) {
    case Phase::LISTENING:
      this->listening_trigger_.trigger();
      break;
    case Phase::USER_SPEAKING:
      this->speech_started_trigger_.trigger();
      break;
    case Phase::THINKING:
      this->speech_stopped_trigger_.trigger();
      break;
    case Phase::SPEAKING:
      this->response_started_trigger_.trigger();
      break;
    default:
      break;
  }
}

void OpenAIRealtime::enqueue_(std::string &&msg) {
  LockGuard guard(this->out_lock_);
  this->out_queue_.push_back(std::move(msg));
}

void OpenAIRealtime::send_function_result(const std::string &call_id, const std::string &output, bool respond) {
  if (this->state_.load() != SessionState::ACTIVE)
    return;
  this->enqueue_(json::build_json([&](JsonObject root) { fill_function_output(root, call_id, output); }));
  if (this->pending_calls_ > 0)
    this->pending_calls_--;
  if (respond)
    this->need_response_create_ = true;
  this->last_activity_ms_ = millis();
  this->maybe_request_response_();
}

void OpenAIRealtime::send_text(const std::string &text) {
  if (this->state_.load() != SessionState::ACTIVE || text.empty())
    return;
  this->enqueue_(json::build_json([&](JsonObject root) { fill_user_text(root, text); }));
  this->need_response_create_ = true;
  this->maybe_request_response_();
}

void OpenAIRealtime::maybe_request_response_() {
  if (!this->need_response_create_ || this->pending_calls_ > 0 || this->response_active_)
    return;
  this->need_response_create_ = false;
  // Marked active now rather than at response.created, so a second result cannot ask twice.
  this->response_active_ = true;
  this->enqueue_(json::build_json([](JsonObject root) { fill_response_create(root); }));
  if (this->phase_ == Phase::LISTENING)
    this->set_phase_(Phase::THINKING);
}

// =================================================================================================
// Main loop
// =================================================================================================

void OpenAIRealtime::loop() {
  if (this->test_requested_.exchange(false)) {
    if (this->state_.load() == SessionState::IDLE) {
      this->start_session_(true);
    } else {
      set_test_result(this->status_lock_, this->test_, false,
                      this->test_only_ ? "A test is already running" : "Finish the conversation first");
    }
  }
  if (this->state_.load() == SessionState::IDLE)
    return;

  if (!this->test_only_ && this->barge_in_pending_.exchange(false))
    this->handle_barge_in_();
  this->process_events_();

  if (this->worker_done_.load()) {
    this->finish_session_();
    return;
  }
  if (!this->test_only_ && this->state_.load() == SessionState::ACTIVE)
    this->feed_speaker_();
  this->check_timeouts_();
}

void OpenAIRealtime::process_events_() {
  for (int i = 0; i < 16; i++) {
    Event ev;
    {
      LockGuard guard(this->event_lock_);
      if (this->events_.empty())
        return;
      ev = std::move(this->events_.front());
      this->events_.pop_front();
    }
    if (this->state_.load() == SessionState::CLOSING && ev.kind != Event::TRANSPORT_ERROR)
      continue;
    this->handle_event_(ev);
  }
}

void OpenAIRealtime::handle_event_(Event &ev) {
  const uint32_t now = millis();
  switch (ev.kind) {
    case Event::CONNECTED:
      ESP_LOGD(TAG, "WebSocket connected");
      break;

    case Event::SESSION_READY:
      if (this->ready_fired_)
        break;  // a later session.updated (we only send one) changes nothing
      this->ready_fired_ = true;
      this->state_.store(SessionState::ACTIVE);
      this->last_activity_ms_ = now;
      if (this->test_only_) {
        const Settings s = this->get_settings();
        const char *flavor = this->flavor_.load() == static_cast<uint8_t>(Flavor::BETA) ? "beta" : "GA";
        std::string msg = "Connected - " + s.model + " over the " + flavor + " Realtime protocol";
        if (ev.a == "assumed")
          msg += " (the server did not confirm the session settings)";
        set_test_result(this->status_lock_, this->test_, true, msg);
        this->request_stop();
        break;
      }
      ESP_LOGI(TAG, "Session ready (%s protocol%s)", this->get_flavor_name(),
               ev.a == "assumed" ? ", settings not confirmed by the server" : "");
      this->ready_trigger_.trigger();
      this->set_phase_(Phase::LISTENING);
      break;

    case Event::SPEECH_STARTED:
      this->last_activity_ms_ = now;
      if (this->phase_ != Phase::SPEAKING)
        this->set_phase_(Phase::USER_SPEAKING);
      break;

    case Event::SPEECH_STOPPED:
      this->last_activity_ms_ = now;
      if (this->phase_ == Phase::USER_SPEAKING || this->phase_ == Phase::LISTENING)
        this->set_phase_(Phase::THINKING);
      break;

    case Event::RESPONSE_CREATED:
      this->response_active_ = true;
      this->last_activity_ms_ = now;
      break;

    case Event::RESPONSE_DONE:
      this->response_active_ = false;
      this->last_activity_ms_ = now;
      if (ev.a == "failed")
        ESP_LOGW(TAG, "Response failed: %s", ev.b.c_str());
      this->maybe_request_response_();
      // A response without audio (only a function call, or cancelled before speaking) ends here.
      if (this->phase_ != Phase::SPEAKING && this->phase_ != Phase::USER_SPEAKING && !this->response_active_ &&
          this->pending_calls_ == 0 && this->playback_ring_ != nullptr && this->playback_ring_->available() == 0 &&
          this->play_len_ == this->play_off_)
        this->set_phase_(Phase::LISTENING);
      break;

    case Event::FUNCTION_CALL:
      this->last_activity_ms_ = now;
      ESP_LOGD(TAG, "Function call %s(%s)", ev.a.c_str(), ev.b.c_str());
      if (this->end_tool_ && ev.a == "end_conversation") {
        // Answered without asking for another response: the goodbye was already spoken (or is
        // playing), and the session closes once it has drained.
        this->enqueue_(json::build_json([&ev](JsonObject root) { fill_function_output(root, ev.c, R"({"ok":true})"); }));
        this->end_after_playback_ = true;
        break;
      }
      if (!this->has_function_handler_) {
        this->pending_calls_++;
        this->send_function_result(ev.c, R"({"error":"No tools are available on this device."})", true);
        break;
      }
      this->pending_calls_++;
      this->function_call_trigger_.trigger(ev.a, ev.b, ev.c);
      break;

    case Event::USER_TRANSCRIPT:
      if (!ev.a.empty())
        this->user_transcript_trigger_.trigger(ev.a);
      break;

    case Event::ASSISTANT_TRANSCRIPT:
      if (!ev.a.empty())
        this->assistant_transcript_trigger_.trigger(ev.a);
      break;

    case Event::SERVER_ERROR:
      // Benign races of the protocol, not failures worth surfacing.
      if (ev.a == "response_cancel_not_active" || ev.a == "conversation_already_has_active_response") {
        ESP_LOGD(TAG, "Server: %s", ev.b.c_str());
        break;
      }
      ESP_LOGW(TAG, "Server error %s: %s", ev.a.c_str(), ev.b.c_str());
      {
        LockGuard guard(this->status_lock_);
        this->last_error_ = ev.b;
      }
      if (this->test_only_ || !this->ready_fired_) {
        // An error before the session is usable (bad model, bad voice, no access) ends it.
        this->fail_(ev.a, ev.b);
      } else {
        this->error_trigger_.trigger(ev.a, ev.b);
      }
      break;

    case Event::TRANSPORT_ERROR:
      if (!this->stop_requested_)
        this->fail_(ev.a, ev.b);
      break;
  }
}

void OpenAIRealtime::feed_speaker_() {
  if (this->playback_ring_ == nullptr || this->play_buf_ == nullptr)
    return;
  // A speaker that is stopping (after a barge-in, or finishing the previous answer) would accept
  // bytes into a buffer it is about to discard. Wait until it has stopped; play() restarts it.
  if (!this->speaker_->is_stopped() && !this->speaker_->is_running())
    return;
  bool accepted_any = false;
  for (int i = 0; i < 8; i++) {
    if (this->play_off_ >= this->play_len_) {
      size_t want = std::min(this->playback_ring_->available(), PLAY_CHUNK) & ~static_cast<size_t>(1);
      if (want == 0)
        break;
      this->play_len_ = this->playback_ring_->read(this->play_buf_, want, 0);
      this->play_off_ = 0;
      if (this->play_len_ == 0)
        break;
    }
    const size_t w = this->speaker_->play(this->play_buf_ + this->play_off_, this->play_len_ - this->play_off_, 0);
    if (w == 0)
      break;
    this->play_off_ += w;
    this->bytes_played_ += w;
    accepted_any = true;
    if (this->play_off_ < this->play_len_)
      break;
  }

  if (accepted_any) {
    // The wall-clock start of the current item's playback, for the truncation estimate.
    const uint32_t off = this->current_item_offset_.load();
    if (off != this->first_play_offset_ && this->bytes_played_ > off) {
      this->first_play_offset_ = off;
      this->item_first_play_ms_ = millis();
    }
    this->last_activity_ms_ = millis();
    this->playing_audio_.store(true);
    if (this->phase_ != Phase::SPEAKING)
      this->set_phase_(Phase::SPEAKING);
    return;
  }

  // Drained: nothing buffered here, nothing in the speaker chain, and the server is done talking.
  if (this->phase_ == Phase::SPEAKING && !this->response_active_ && this->playback_ring_->available() == 0 &&
      this->play_off_ >= this->play_len_ && !this->speaker_->has_buffered_data()) {
    this->playing_audio_.store(false);
    this->speaker_->finish();
    this->last_activity_ms_ = millis();
    this->response_finished_trigger_.trigger();
    this->set_phase_(this->pending_calls_ > 0 || this->need_response_create_ ? Phase::THINKING : Phase::LISTENING);
  }
}

void OpenAIRealtime::handle_barge_in_() {
  std::string item;
  uint32_t offset;
  {
    LockGuard guard(this->item_lock_);
    item = this->barge_item_;
    offset = this->barge_offset_;
  }
  const bool audible = this->playing_audio_.load() || this->play_off_ < this->play_len_ ||
                       (this->playback_ring_ != nullptr && this->playback_ring_->available() > 0);
  if (!audible)
    return;

  // How much of the item the person actually heard: what the speaker chain accepted, bounded by the
  // wall clock since it started, minus what is still queued in the chain (playback_latency).
  const uint32_t now = millis();
  const uint32_t played = this->bytes_played_ > offset ? this->bytes_played_ - offset : 0;
  uint32_t heard = played / BYTES_PER_MS;
  if (this->item_first_play_ms_ != 0 && this->first_play_offset_ == offset)
    heard = std::min(heard, now - this->item_first_play_ms_);
  heard = heard > this->playback_latency_ms_ ? heard - this->playback_latency_ms_ : 0;

  // Silence it now: the speaker chain is flushed, and whatever was queued here is dropped. Deltas of
  // this item still on the wire are discarded on the WebSocket task (dropped_item_).
  this->speaker_->stop();
  this->playback_ring_->reset();
  this->play_len_ = this->play_off_ = 0;
  this->playing_audio_.store(false);
  // Everything written so far has now left: played, or discarded just above. Without this the
  // discarded bytes would sit between bytes_played_ and every later item's offset.
  this->bytes_played_ = this->bytes_written_.load();
  ESP_LOGI(TAG, "Interrupted after ~%" PRIu32 " ms of %s", heard, item.c_str());

  if (!item.empty())
    this->enqueue_(json::build_json([&](JsonObject root) { fill_truncate(root, item, heard); }));
  this->interrupted_trigger_.trigger();
  this->set_phase_(Phase::USER_SPEAKING);
}

void OpenAIRealtime::check_timeouts_() {
  const SessionState st = this->state_.load();
  if (st == SessionState::CLOSING)
    return;
  const uint32_t now = millis();
  if (st == SessionState::CONNECTING && now - this->session_started_ms_ > CONNECT_TIMEOUT_MS) {
    this->fail_("timeout", "Timed out connecting to the Realtime server");
    return;
  }
  if (this->test_only_ || st != SessionState::ACTIVE)
    return;
  if (this->muted_fn_ && this->muted_fn_()) {
    ESP_LOGI(TAG, "Microphones muted - ending the conversation");
    this->request_stop();
    return;
  }
  if (this->max_duration_ms_ > 0 && now - this->session_started_ms_ > this->max_duration_ms_) {
    ESP_LOGI(TAG, "Maximum conversation length reached");
    this->request_stop();
    return;
  }
  const bool quiet = !this->response_active_ && this->pending_calls_ == 0 && !this->need_response_create_ &&
                     !this->playing_audio_.load() && this->playback_ring_->available() == 0 &&
                     this->play_off_ >= this->play_len_;
  if (this->end_after_playback_ && quiet && this->phase_ != Phase::SPEAKING) {
    ESP_LOGI(TAG, "Conversation ended by the assistant");
    this->request_stop();
    return;
  }
  if (this->idle_timeout_ms_ > 0 && quiet && this->phase_ == Phase::LISTENING &&
      now - this->last_activity_ms_ > this->idle_timeout_ms_) {
    ESP_LOGI(TAG, "No speech for %" PRIu32 " s - ending the conversation", this->idle_timeout_ms_ / 1000);
    this->request_stop();
  }
}

// =================================================================================================
// Microphone task
// =================================================================================================

void OpenAIRealtime::on_mic_data_(const std::vector<uint8_t> &data) {
  if (!this->accepting_mic_.load() || this->mic_ring_ == nullptr)
    return;
  const size_t n = data.size() / 2;
  if (n == 0)
    return;
  const size_t need = Upsampler2to3::max_output(n);
  if (this->up_buf_.size() < need)
    this->up_buf_.resize(need);
  const size_t produced =
      this->upsampler_.process(reinterpret_cast<const int16_t *>(data.data()), n, this->up_buf_.data());
  // Never blocks the microphone task; a stalled uplink loses the oldest audio, not the newest.
  this->mic_ring_->write(this->up_buf_.data(), produced * 2);
}

// =================================================================================================
// Worker task
// =================================================================================================

void OpenAIRealtime::worker_task_(void *arg) {
  auto *self = static_cast<OpenAIRealtime *>(arg);
  self->run_worker_();
  self->worker_done_.store(true);
  App.wake_loop_threadsafe();
  vTaskDelete(nullptr);
}

void OpenAIRealtime::run_worker_() {
  esp_websocket_client_config_t cfg{};
  cfg.uri = this->ws_url_.c_str();
  cfg.task_stack = 6144;  // TLS handshake runs on the client's own task
  cfg.task_prio = 5;
  // 16 KB, so session.update (instructions + tools) normally leaves as one frame; larger messages
  // are sent as continuation frames, which the protocol allows.
  cfg.buffer_size = 16384;
  cfg.network_timeout_ms = 10000;
  cfg.disable_auto_reconnect = true;
  cfg.reconnect_timeout_ms = 10000;
  cfg.ping_interval_sec = 20;
  cfg.pingpong_timeout_sec = 30;
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
  if (this->ws_url_.rfind("wss://", 0) == 0)
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
#endif

  this->client_ = esp_websocket_client_init(&cfg);
  if (this->client_ == nullptr) {
    this->push_event_({Event::TRANSPORT_ERROR, "no_memory", "Could not create the WebSocket client"});
    return;
  }
  if (!this->auth_header_.empty())
    esp_websocket_client_append_header(this->client_, "Authorization", this->auth_header_.c_str());
  // api.openai.com serves the beta format only with this header; other servers ignore it.
  if (this->forced_flavor_ == Flavor::BETA)
    esp_websocket_client_append_header(this->client_, "OpenAI-Beta", "realtime=v1");
  for (const auto &h : this->extra_headers_)
    esp_websocket_client_append_header(this->client_, h.first.c_str(), h.second.c_str());
  esp_websocket_register_events(this->client_, WEBSOCKET_EVENT_ANY, &OpenAIRealtime::ws_event_handler_, this);

  if (esp_websocket_client_start(this->client_) != ESP_OK) {
    this->push_event_({Event::TRANSPORT_ERROR, "connect", "Could not start the WebSocket connection"});
  } else {
    // Wait for the handshake.
    const uint32_t t0 = millis();
    while (!this->worker_stop_.load() && !this->ws_connected_.load() && !this->transport_failed_.load() &&
           millis() - t0 < CONNECT_TIMEOUT_MS)
      vTaskDelay(pdMS_TO_TICKS(20));

    if (this->ws_connected_.load() && !this->worker_stop_.load()) {
      // session.created says which format the server speaks. A server that sends none gets GA,
      // unless the YAML forced a format.
      const uint32_t t1 = millis();
      while (!this->session_created_.load() && !this->worker_stop_.load() && this->ws_connected_.load() &&
             millis() - t1 < SESSION_CREATED_WAIT_MS)
        vTaskDelay(pdMS_TO_TICKS(10));
      Flavor flavor = this->forced_flavor_;
      if (flavor == Flavor::UNKNOWN)
        flavor = static_cast<Flavor>(this->flavor_.load());
      if (flavor == Flavor::UNKNOWN)
        flavor = Flavor::GA;
      this->flavor_.store(static_cast<uint8_t>(flavor));
      const std::string &update = flavor == Flavor::BETA ? this->session_update_beta_ : this->session_update_;
      ESP_LOGD(TAG, "Configuring the session (%s protocol)", flavor == Flavor::BETA ? "beta" : "GA");

      const uint32_t connected_ms = millis();
      bool ok = this->send_raw_(update.data(), update.size());
      while (ok && !this->worker_stop_.load() && this->ws_connected_.load()) {
        // Control messages first: a truncate or a function result must not queue behind audio.
        for (int i = 0; i < 4 && ok; i++) {
          std::string msg;
          {
            LockGuard guard(this->out_lock_);
            if (this->out_queue_.empty())
              break;
            msg = std::move(this->out_queue_.front());
            this->out_queue_.pop_front();
          }
          ok = this->send_raw_(msg.data(), msg.size());
        }
        if (!ok)
          break;
        // Audio waits for session.updated so it lands in the configured format. A server that
        // never confirms (and has not sent an error) is taken as configured after the grace period;
        // PCM 24 kHz is every Realtime server's default anyway.
        if (!this->session_configured_.load() && millis() - connected_ms >= CONFIGURE_GRACE_MS) {
          this->session_configured_.store(true);
          this->push_event_({Event::SESSION_READY, "assumed", ""});
        }
        if (this->test_only_ || !this->session_configured_.load()) {
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }
        ok = this->send_mic_audio_();
      }
    } else if (!this->worker_stop_.load() && !this->transport_failed_.load()) {
      this->push_event_({Event::TRANSPORT_ERROR, "timeout", "Timed out connecting to the Realtime server"});
    }
  }

  // Teardown on this task, never the WebSocket task's own: close() and destroy() join that task.
  if (esp_websocket_client_is_connected(this->client_)) {
    esp_websocket_client_close(this->client_, pdMS_TO_TICKS(1500));
  }
  esp_websocket_client_destroy(this->client_);
  this->client_ = nullptr;
  this->ws_connected_.store(false);
}

bool OpenAIRealtime::send_raw_(const char *data, size_t len) {
  if (this->client_ == nullptr || !esp_websocket_client_is_connected(this->client_))
    return false;
  const int r = esp_websocket_client_send_text(this->client_, data, static_cast<int>(len), pdMS_TO_TICKS(3000));
  if (r < 0 || static_cast<size_t>(r) != len) {
    if (!this->worker_stop_.load())
      this->push_event_({Event::TRANSPORT_ERROR, "send", "Sending to the Realtime server failed"});
    return false;
  }
  return true;
}

bool OpenAIRealtime::send_mic_audio_() {
  // Waits up to 40 ms for a full 60 ms chunk, so this loop paces itself on the microphone.
  size_t n = this->mic_ring_->read(this->pcm_buf_, MIC_CHUNK, pdMS_TO_TICKS(40));
  n &= ~static_cast<size_t>(1);
  if (n == 0)
    return true;
  if (this->half_duplex_ && this->playing_audio_.load())
    return true;  // half duplex: the device does not listen while it talks

  // The one message not built with ArduinoJson (see rt_messages.h): base64 straight into the
  // reused frame buffer.
  const size_t total = build_audio_append(this->pcm_buf_, n, this->tx_buf_, this->tx_cap_);
  if (total == 0)
    return true;
  return this->send_raw_(this->tx_buf_, total);
}

// =================================================================================================
// WebSocket task
// =================================================================================================

void OpenAIRealtime::push_event_(Event &&ev) {
  {
    LockGuard guard(this->event_lock_);
    // Bounded: a loop stalled for seconds must not grow this without limit.
    if (this->events_.size() < 64)
      this->events_.push_back(std::move(ev));
  }
  App.wake_loop_threadsafe();
}

void OpenAIRealtime::ws_event_handler_(void *arg, esp_event_base_t base, int32_t id, void *data) {
  auto *self = static_cast<OpenAIRealtime *>(arg);
  auto *d = static_cast<esp_websocket_event_data_t *>(data);
  switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
      self->ws_connected_.store(true);
      self->push_event_({Event::CONNECTED, "", ""});
      break;
    case WEBSOCKET_EVENT_DATA:
      if (d != nullptr)
        self->on_ws_data_(d);
      break;
    case WEBSOCKET_EVENT_ERROR: {
      if (self->worker_stop_.load())
        break;
      const int status = d != nullptr ? d->error_handle.esp_ws_handshake_status_code : 0;
      std::string msg;
      std::string code = "connect";
      if (status == 401) {
        msg = "The server rejected the API key (HTTP 401)";
        code = "unauthorized";
      } else if (status == 403) {
        msg = "This key has no access to the Realtime API (HTTP 403)";
        code = "forbidden";
      } else if (status == 404) {
        msg = "No Realtime endpoint at this address (HTTP 404)";
        code = "not_found";
      } else if (status != 0) {
        msg = "The server refused the WebSocket upgrade (HTTP " + to_string(status) + ")";
      } else if (!self->ws_connected_.load()) {
        msg = "Could not reach the Realtime server";
      } else {
        msg = "The connection to the Realtime server failed";
      }
      self->transport_failed_.store(true);
      self->push_event_({Event::TRANSPORT_ERROR, code, msg});
      break;
    }
    case WEBSOCKET_EVENT_DISCONNECTED:
    case WEBSOCKET_EVENT_CLOSED:
      self->ws_connected_.store(false);
      App.wake_loop_threadsafe();
      break;
    default:
      break;
  }
}

void OpenAIRealtime::on_ws_data_(const esp_websocket_event_data_t *d) {
  const uint8_t op = d->op_code;
  if (op == 0x08) {
    // Close frame: a two-byte code and a reason, e.g. a server-side session limit.
    if (d->payload_offset == 0 && d->data_len >= 2 && d->data_ptr != nullptr) {
      const int code = (static_cast<uint8_t>(d->data_ptr[0]) << 8) | static_cast<uint8_t>(d->data_ptr[1]);
      std::string reason(d->data_ptr + 2, d->data_len - 2);
      if (!this->worker_stop_.load() && code != 1000)
        this->close_reason_ = "The server closed the connection (" + to_string(code) +
                              (reason.empty() ? std::string(")") : "): " + reason);
    }
    return;
  }
  if (op == 0x09 || op == 0x0A)
    return;  // ping / pong, handled by the client
  if (op != 0x00 && op != 0x01 && op != 0x02)
    return;

  // A new message begins at offset 0 of a text/binary frame; continuation frames (op 0) append.
  if (op != 0x00 && d->payload_offset == 0) {
    this->rx_len_ = 0;
    this->rx_overflow_ = false;
  }
  if (d->data_len > 0 && d->data_ptr != nullptr) {
    if (this->rx_len_ + d->data_len > this->rx_cap_) {
      this->rx_overflow_ = true;
    } else if (!this->rx_overflow_) {
      memcpy(this->rx_buf_ + this->rx_len_, d->data_ptr, d->data_len);
      this->rx_len_ += d->data_len;
    }
  }
  const bool frame_done = d->payload_offset + d->data_len >= d->payload_len;
  if (frame_done && d->fin) {
    if (this->rx_overflow_) {
      ESP_LOGW(TAG, "Dropped a server event larger than %u bytes", (unsigned) this->rx_cap_);
    } else if (this->rx_len_ > 0) {
      this->on_ws_message_(this->rx_buf_, this->rx_len_);
    }
    this->rx_len_ = 0;
    this->rx_overflow_ = false;
  }
}

void OpenAIRealtime::on_audio_delta_(char *buf, size_t len) {
  if (this->test_only_ || this->playback_ring_ == nullptr)
    return;
  size_t ds, dl, is, il;
  if (!find_json_string(buf, len, "delta", ds, dl))
    return;
  std::string item;
  if (find_json_string(buf, len, "item_id", is, il))
    item.assign(buf + is, il);
  {
    LockGuard guard(this->item_lock_);
    if (!item.empty() && item == this->dropped_item_)
      return;  // the person talked over this one; its remaining audio is not wanted
    if (item != this->current_item_) {
      this->current_item_ = item;
      this->current_item_offset_.store(this->bytes_written_.load());
    }
  }
  bool ok;
  uint8_t *pcm = reinterpret_cast<uint8_t *>(buf + ds);
  const size_t n = base64_decode(buf + ds, dl, pcm, ok);
  if (!ok)
    ESP_LOGW(TAG, "Malformed audio in a delta");
  // Backpressure only if the playback ring (seconds deep) is full; the loop drains it in real time.
  size_t off = 0;
  while (off < n && !this->worker_stop_.load()) {
    const size_t w = this->playback_ring_->write_without_replacement(pcm + off, n - off, pdMS_TO_TICKS(50));
    off += w;
    this->bytes_written_.fetch_add(static_cast<uint32_t>(w));
  }
  App.wake_loop_threadsafe();
}

void OpenAIRealtime::on_ws_message_(char *buf, size_t len) {
  const std::string type = first_type(buf, len);
  // GA spells the output events response.output_audio.*, the beta format response.audio.*.
  if (type == "response.output_audio.delta" || type == "response.audio.delta") {
    this->on_audio_delta_(buf, len);
    return;
  }
  // High-rate events nobody acts on; skipping them saves a parse each.
  if (type == "response.output_audio_transcript.delta" || type == "response.audio_transcript.delta" ||
      type == "response.output_text.delta" || type == "response.text.delta" ||
      type == "response.function_call_arguments.delta" || type == "conversation.item.input_audio_transcription.delta" ||
      type == "rate_limits.updated" || type == "response.content_part.added" || type == "response.content_part.done")
    return;
  if (type == "session.created") {
    const Flavor f = detect_flavor(buf, len);
    if (f != Flavor::UNKNOWN && this->forced_flavor_ == Flavor::UNKNOWN)
      this->flavor_.store(static_cast<uint8_t>(f));
    this->session_created_.store(true);
    return;
  }

  if (type == "input_audio_buffer.speech_started") {
    // Barge-in is decided here, on the task that writes audio, so the very next delta of the
    // interrupted item is already discarded. The loop silences what is queued.
    {
      LockGuard guard(this->item_lock_);
      const bool audio_pending =
          this->playing_audio_.load() || this->ws_response_active_.load() ||
          (this->playback_ring_ != nullptr && this->playback_ring_->available() > 0);
      if (!this->current_item_.empty() && audio_pending && this->current_item_ != this->dropped_item_) {
        this->dropped_item_ = this->current_item_;
        this->barge_item_ = this->current_item_;
        this->barge_offset_ = this->current_item_offset_.load();
        this->current_item_.clear();
        this->barge_in_pending_.store(true);
      }
    }
    this->push_event_({Event::SPEECH_STARTED, "", ""});
    return;
  }
  if (type == "input_audio_buffer.speech_stopped") {
    this->push_event_({Event::SPEECH_STOPPED, "", ""});
    return;
  }

  JsonDocument doc = json::parse_json(reinterpret_cast<const uint8_t *>(buf), len);
  JsonObjectConst root = doc.as<JsonObjectConst>();
  if (root.isNull())
    return;

  if (type == "session.updated") {
    this->session_configured_.store(true);
    this->push_event_({Event::SESSION_READY, "", ""});
  } else if (type == "response.created") {
    this->ws_response_active_.store(true);
    this->push_event_({Event::RESPONSE_CREATED, "", ""});
  } else if (type == "response.done") {
    this->ws_response_active_.store(false);
    JsonObjectConst r = root["response"];
    std::string status = r["status"] | "";
    std::string detail;
    if (status == "failed") {
      detail = r["status_details"]["error"]["message"] | "";
    }
    this->push_event_({Event::RESPONSE_DONE, status, detail});
  } else if (type == "response.output_item.done") {
    JsonObjectConst item = root["item"];
    if (std::string(item["type"] | "") == "function_call") {
      this->push_event_({Event::FUNCTION_CALL, item["name"] | "", item["arguments"] | "{}", item["call_id"] | ""});
    }
  } else if (type == "conversation.item.input_audio_transcription.completed") {
    this->push_event_({Event::USER_TRANSCRIPT, root["transcript"] | "", ""});
  } else if (type == "response.output_audio_transcript.done" || type == "response.audio_transcript.done") {
    this->push_event_({Event::ASSISTANT_TRANSCRIPT, root["transcript"] | "", ""});
  } else if (type == "error") {
    JsonObjectConst e = root["error"];
    std::string code = e["code"] | "";
    if (code.empty())
      code = e["type"] | "error";
    this->push_event_({Event::SERVER_ERROR, code, e["message"] | "Unknown error"});
  }
}

// =================================================================================================
// /models
// =================================================================================================

uint32_t OpenAIRealtime::request_models(const std::string &base_url, const std::string &api_key) {
  const Settings s = this->get_settings();
  std::string key = api_key;
  const std::string origin = url_origin(base_url);
  if (key.empty() && !origin.empty() && origin == url_origin(s.base_url))
    key = s.api_key;

  bool start_task = false;
  uint32_t gen;
  {
    LockGuard guard(this->models_lock_);
    gen = ++this->models_req_gen_;
    this->models_req_base_ = normalize_base_url(base_url);
    this->models_req_key_ = key;
    this->models_.gen = gen;
    this->models_.state = 1;
    this->models_.http_status = 0;
    this->models_.base_url = this->models_req_base_;
    this->models_.error.clear();
    this->models_.has_model_list = false;
    this->models_.ids.clear();
    this->models_.voices.clear();
    this->models_.voices_from_server = 0;
    if (!this->models_task_running_) {
      this->models_task_running_ = true;
      start_task = true;
    }
  }
  if (start_task && xTaskCreate(&OpenAIRealtime::models_task_, "oai_models", 8192, this, 3, nullptr) != pdPASS) {
    LockGuard guard(this->models_lock_);
    this->models_task_running_ = false;
    this->models_.state = 3;
    this->models_.error = "Not enough memory to fetch the model list";
  }
  return gen;
}

ModelsStatus OpenAIRealtime::get_models_status() const {
  LockGuard guard(this->models_lock_);
  return this->models_;
}

void OpenAIRealtime::models_task_(void *arg) {
  static_cast<OpenAIRealtime *>(arg)->run_models_();
  vTaskDelete(nullptr);
}

static std::string fetch_error_message(int status, const char *body, size_t len) {
  size_t s, n;
  std::string server;
  if (body != nullptr && find_json_string(body, len, "message", s, n) && n < 200)
    server.assign(body + s, n);
  std::string msg;
  switch (status) {
    case 401:
      msg = "The server rejected the API key (HTTP 401)";
      break;
    case 403:
      msg = "This key may not list models (HTTP 403)";
      break;
    default:
      msg = "The server answered HTTP " + to_string(status);
  }
  if (!server.empty())
    msg += ": " + server;
  return msg;
}

/// One GET into `body` (NUL-terminated). Returns the HTTP status, or 0 with `error` set when the
/// server could not be reached at all.
int OpenAIRealtime::http_get_(const std::string &url, bool secure, const std::string &key, char *body, size_t cap,
                              size_t &len, std::string &error) {
  len = 0;
  esp_http_client_config_t cfg{};
  cfg.url = url.c_str();
  cfg.method = HTTP_METHOD_GET;
  cfg.timeout_ms = 10000;
  cfg.buffer_size = 2048;
  cfg.buffer_size_tx = 1024;
  cfg.max_redirection_count = 3;
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
  if (secure)
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
#endif
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (client == nullptr) {
    error = "Could not create the HTTP client";
    return 0;
  }
  if (!key.empty())
    esp_http_client_set_header(client, "Authorization", ("Bearer " + key).c_str());
  for (const auto &h : this->extra_headers_)
    esp_http_client_set_header(client, h.first.c_str(), h.second.c_str());
  int status = 0;
  const esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    error = std::string("Could not reach the server (") + esp_err_to_name(err) + ")";
  } else {
    esp_http_client_fetch_headers(client);
    status = esp_http_client_get_status_code(client);
    while (len < cap - 1) {
      const int r = esp_http_client_read(client, body + len, static_cast<int>(cap - 1 - len));
      if (r <= 0)
        break;
      len += static_cast<size_t>(r);
    }
  }
  body[len] = '\0';
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  return status;
}

void OpenAIRealtime::run_models_() {
  char *body = static_cast<char *>(psram_alloc(MODELS_BODY_MAX));
  while (true) {
    std::string base, key;
    uint32_t gen;
    {
      LockGuard guard(this->models_lock_);
      base = this->models_req_base_;
      key = this->models_req_key_;
      gen = this->models_req_gen_;
    }

    ModelsStatus r;
    Endpoints ep;
    std::string why;
    if (body == nullptr) {
      r.state = 3;
      r.error = "Not enough memory to fetch the model list";
    } else if (!derive_endpoints(base, "", ep, why)) {
      r.state = 3;
      r.error = "That is not a valid http(s):// or ws(s):// address";
    } else {
      // 1. /models proves the address and the key, and lists the models.
      size_t len = 0;
      std::string err;
      r.http_status = this->http_get_(ep.models_url, ep.secure, key, body, MODELS_BODY_MAX, len, err);
      if (r.http_status == 0) {
        r.state = 3;
        r.error = err + " at " + ep.origin;
      } else if (r.http_status == 200) {
        collect_model_ids(body, len, r.ids);
        filter_realtime_models(r.ids);
        r.has_model_list = !r.ids.empty();
        r.state = 2;
      } else if (r.http_status == 404 || r.http_status == 405 || r.http_status == 501) {
        // Reachable, but no model list here: a local server that serves one model. The key is not
        // proven, but nothing more can be learned without opening a session (Test connection).
        r.state = 2;
      } else {
        r.state = 3;
        r.error = fetch_error_message(r.http_status, body, len);
      }

      // 2. /audio/voices. OpenAI lists only an organisation's custom voices there (and refuses keys
      // without access); local servers list theirs. Failure is not an error - the built-ins stay.
      if (r.state == 2) {
        const bool openai = ep.origin == OPENAI_ORIGIN;
        if (openai) {
          for (size_t i = 0; i < BUILTIN_VOICE_COUNT; i++)
            r.voices.push_back({BUILTIN_VOICES[i], ""});
        }
        std::vector<VoiceEntry> listed;
        const int vs = this->http_get_(ep.voices_url, ep.secure, key, body, VOICES_BODY_MAX, len, err);
        if (vs == 200)
          collect_voices(body, len, listed);
        for (auto &v : listed) {
          bool dup = false;
          for (const auto &e : r.voices)
            dup = dup || e.id == v.id;
          if (!dup && v.id.size() <= MAX_VOICE) {
            r.voices.push_back(std::move(v));
            r.voices_from_server++;
          }
        }
        ESP_LOGD(TAG, "Voices for %s: %u listed by the server (HTTP %d)", base.c_str(), r.voices_from_server, vs);
      }
    }
    key.assign(key.size(), '\0');

    LockGuard guard(this->models_lock_);
    if (gen != this->models_req_gen_)
      continue;  // superseded while we were fetching: fetch the newer one
    this->models_.state = r.state;
    this->models_.http_status = r.http_status;
    this->models_.error = r.error;
    this->models_.has_model_list = r.has_model_list;
    this->models_.ids = std::move(r.ids);
    this->models_.voices = std::move(r.voices);
    this->models_.voices_from_server = r.voices_from_server;
    this->models_task_running_ = false;
    ESP_LOGD(TAG, "Discovery for %s: %s", base.c_str(), this->models_.error.empty() ? "ok" : this->models_.error.c_str());
    break;
  }
  heap_caps_free(body);
}

}  // namespace openai_realtime
}  // namespace esphome

#endif  // USE_ESP32
