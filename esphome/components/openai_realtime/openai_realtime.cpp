#include "openai_realtime.h"

#ifdef USE_ESP32

#include <cctype>
#include <cinttypes>
#include <cstring>

#include <esp_heap_caps.h>
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
// characters (3.9 KB with the envelope), so every append fits the 4 KB client buffer as one frame.
// Only whole chunks are sent: the microphone delivers ~16 ms at a time, and a frame per delivery
// would quadruple the message rate (and the small TCP writes Nagle holds back).
static constexpr size_t MIC_CHUNK = 60 * BYTES_PER_MS;
static constexpr size_t MIC_RING_SIZE = 256 * 1024;  // ~5 s: covers the connect, so the first words are kept
static constexpr size_t RX_BUF_SIZE = 512 * 1024;    // largest server event we accept (audio deltas)
static constexpr size_t TX_BUF_SIZE = 8 * 1024;
static constexpr int WS_BUFFER_SIZE = 4096;          // the client's own rx and tx buffers (internal RAM)
static constexpr uint32_t WORKER_STACK_SIZE = 8192;  // PSRAM
static constexpr size_t PLAY_CHUNK = 4096;
static constexpr uint32_t CONNECT_TIMEOUT_MS = 20000;
// A send that cannot complete in this long ends the session (the client drops the connection on a
// send timeout). Long, because a server may stop reading for a few seconds while it is busy (e.g.
// while it plays an answer); the session should ride that out, not fail.
static constexpr uint32_t SEND_TIMEOUT_MS = 10000;
// A microphone frame that took this long to send means the server stopped reading for a while: the
// audio queued meanwhile is stale and is dropped rather than sent seconds late.
static constexpr uint32_t SEND_STALL_MS = 1000;
// Audio buffered before an answer starts playing, against gaps from a server that sends audio
// paced in real time (or network jitter). Shorter answers start as soon as the server is done.
static constexpr size_t PREBUFFER_BYTES = 150 * BYTES_PER_MS;
// A server that never sends response.done: its answer counts as finished once its audio has played
// and nothing about the response has arrived for this long.
static constexpr uint32_t RESPONSE_SILENCE_MS = 5000;
// The assistant-requested end (end_conversation) stops the session once everything has drained
// AND stayed quiet for this long. The debounce covers a farewell the server starts speaking via
// its own TTS right after the tool-call response's response.done: without it, the quiet window
// between that done and the utterance's first audio would end the session mid-goodbye.
static constexpr uint32_t END_QUIET_DEBOUNCE_MS = 300;
// Turn-based mode: after an answer has played, the microphone stays closed this much longer, so the
// tail of the speaker (I2S, room echo) is not sent as the person's next turn.
static constexpr uint32_t TURN_TAIL_MS = 500;
// "Thinking" with no response, function call or audio in progress for this long means the server
// let the turn go (an error, or a turn too short to answer): back to listening, so a turn-based
// session does not sit with its microphone closed.
static constexpr uint32_t THINKING_TIMEOUT_MS = 8000;

// After session.update, a server that never confirms with session.updated (several self-hosted ones)
// is taken as configured after this long without an error. OpenAI confirms within a few hundred ms;
// audio is buffered meanwhile, so this only delays the first answer.
static constexpr uint32_t CONFIGURE_GRACE_MS = 1500;
// How long to wait after the WebSocket opens for session.created, which tells GA from beta.
static constexpr uint32_t SESSION_CREATED_WAIT_MS = 2500;
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
  if (this->mic_source_ != nullptr) {
    this->mic_source_->add_data_callback([this](const std::vector<uint8_t> &data) { this->on_mic_data_(data); });
  }
  this->up_buf_.reserve(4096);
}

void OpenAIRealtime::dump_config() {
  Endpoints ep;
  std::string err;
  const bool ok = derive_endpoints(this->base_url_, this->model_, ep, err);
  ESP_LOGCONFIG(TAG,
                "OpenAI Realtime:\n"
                "  Base URL: %s\n"
                "  Realtime URL: %s\n"
                "  Model: %s\n"
                "  Voice: %s\n"
                "  API key: %s\n"
                "  Turn detection: %s\n"
                "  Duplex: %s\n"
                "  Idle timeout: %" PRIu32 " ms",
                this->base_url_.c_str(), ok ? ep.realtime_url.c_str() : "(invalid)", this->model_.c_str(),
                this->voice_.c_str(), this->api_key_.empty() ? "not set" : "set", this->td_type_.c_str(),
                this->full_duplex_.load() ? "full (barge-in)" : "turn-based", this->idle_timeout_ms_);
}

bool OpenAIRealtime::is_configured() const {
  // Validated at build time already (__init__.py); this guards against a lambda-set value.
  Endpoints ep;
  std::string err;
  if (!derive_endpoints(this->base_url_, this->model_, ep, err) || this->model_.empty())
    return false;
  // A local server may need no key; api.openai.com never accepts an unauthenticated session.
  return !this->api_key_.empty() || ep.origin != OPENAI_ORIGIN;
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
    this->fail_("not_configured", "OpenAI Realtime is not configured - check base_url, model and api_key");
    return;
  }
  this->start_session_();
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
  if (this->mic_source_ != nullptr)
    this->mic_source_->stop();
  if (this->speaker_ != nullptr)
    this->speaker_->stop();
  this->playing_audio_.store(false);
}

void OpenAIRealtime::fail_(const std::string &code, const std::string &message) {
  ESP_LOGW(TAG, "%s: %s", code.c_str(), message.c_str());
  this->error_trigger_.trigger(code, message);
  this->request_stop();
}

void OpenAIRealtime::build_session_update_() {
  // Both shapes are built up front; the worker sends the one the server's session.created asks for.
  SessionConfig c;
  c.model = this->model_;
  c.voice = this->voice_;
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
  c.interrupt_response = this->session_full_duplex_.load();
  this->session_update_ = json::build_json([&c](JsonObject root) { fill_session_update(root, c, Flavor::GA); });
  this->session_update_beta_ = json::build_json([&c](JsonObject root) { fill_session_update(root, c, Flavor::BETA); });
}

void OpenAIRealtime::start_session_() {
  Endpoints ep;
  std::string why;
  if (!derive_endpoints(this->base_url_, this->model_, ep, why)) {
    this->fail_("bad_url", "The base URL is not a valid http(s):// or ws(s):// address");
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
  if (this->mic_ring_ == nullptr)
    this->mic_ring_ = ring_buffer::RingBuffer::create(MIC_RING_SIZE);
  if (this->playback_ring_ == nullptr)
    this->playback_ring_ = ring_buffer::RingBuffer::create(this->playback_buffer_size_);
  if (this->rx_buf_ == nullptr || this->tx_buf_ == nullptr || this->pcm_buf_ == nullptr || this->play_buf_ == nullptr ||
      this->mic_ring_ == nullptr || this->playback_ring_ == nullptr) {
    this->release_buffers_();
    this->fail_("no_memory", "Not enough memory to start a conversation");
    return;
  }

  this->ws_url_ = ep.realtime_url;
  this->auth_header_ = this->api_key_.empty() ? "" : "Bearer " + this->api_key_;
  // The mode is fixed for the session: session.update, the mic gate and barge-in must agree.
  this->session_full_duplex_.store(this->full_duplex_.load());
  this->mic_open_.store(true);
  this->mic_reopen_at_ = millis();
  this->build_session_update_();

  // Session flags.
  this->stop_requested_ = false;
  this->response_active_ = false;
  this->need_response_create_ = false;
  this->pending_calls_ = 0;
  this->end_after_playback_ = false;
  this->end_quiet_since_ = 0;
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
  this->last_response_ms_.store(0);
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
  this->close_reason_.clear();
  this->worker_stop_.store(false);
  this->worker_done_.store(false);
  this->ws_connected_.store(false);
  this->session_configured_.store(false);
  this->session_created_.store(false);
  this->flavor_.store(static_cast<uint8_t>(Flavor::UNKNOWN));
  this->transport_failed_.store(false);

  this->mic_ring_->reset();
  this->playback_ring_->reset();
  this->upsampler_.reset();
  this->speaker_->stop();
  this->speaker_->set_audio_stream_info(audio::AudioStreamInfo(16, 1, 24000));
  this->accepting_mic_.store(true);
  this->mic_source_->start();

  this->state_.store(SessionState::CONNECTING);
  // The worker owns the socket for the session's life. Its stack is in PSRAM: internal RAM is scarce
  // on this board, and mbedTLS allocates externally here (CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC).
  ESP_LOGD(TAG, "Internal RAM: %u bytes free, largest block %u",
           static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
           static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
  if (!this->worker_.create(&OpenAIRealtime::worker_task_, "oai_rt", WORKER_STACK_SIZE, this, 5, true)) {
    this->state_.store(SessionState::IDLE);
    this->accepting_mic_.store(false);
    this->mic_source_->stop();
    this->fail_("no_memory", "Could not start the connection task");
    return;
  }
  this->high_freq_.start();
  ESP_LOGI(TAG, "Connecting to %s (%s)", this->ws_url_.c_str(),
           this->session_full_duplex_.load() ? "full duplex" : "turn-based");
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
  // Expected ends: this client requested the stop (wake word, button, mute, max duration,
  // idle timeout) - or the assistant's end_conversation flow completed, where the server's own
  // graceful close fires the moment the farewell audio drains and naturally beats this side's
  // end debounce. A close carrying a reason (a non-1000 code) is never expected.
  const bool assistant_ended_cleanly = this->end_after_playback_ && this->close_reason_.empty();
  const bool unexpected = !this->stop_requested_ && !assistant_ended_cleanly;
  this->accepting_mic_.store(false);
  if (this->mic_source_ != nullptr)
    this->mic_source_->stop();
  if (this->speaker_ != nullptr)
    this->speaker_->stop();
  this->playing_audio_.store(false);
  this->release_buffers_();
  this->high_freq_.stop();
  this->phase_ = Phase::NONE;
  this->state_.store(SessionState::IDLE);

  if (unexpected) {
    const std::string msg = this->close_reason_.empty() ? "The server closed the connection" : this->close_reason_;
    ESP_LOGW(TAG, "%s", msg.c_str());
    this->error_trigger_.trigger("disconnected", msg);
  }
  this->end_trigger_.trigger();
  ESP_LOGI(TAG, "Session ended");
}

void OpenAIRealtime::set_phase_(Phase p) {
  if (this->phase_ == p)
    return;
  // Turn-based: back to listening after the model's turn - hold the mic closed a little longer.
  if (p == Phase::LISTENING && (this->phase_ == Phase::SPEAKING || this->phase_ == Phase::THINKING))
    this->mic_reopen_at_ = millis() + TURN_TAIL_MS;
  // Sync trace (turn-based LED lag): every phase maps to a trigger the LED
  // animations hook, so this line IS the LED timeline.  Pair it with the
  // drain trace in check_timeouts_ to see where the 0.5-1 s goes.
  ESP_LOGD(TAG, "Phase: %s -> %s (t=%u ms)", phase_name(this->phase_), phase_name(p), millis());
  this->phase_ = p;
  this->phase_since_ms_ = millis();
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
  this->last_response_ms_.store(millis());
  this->enqueue_(json::build_json([](JsonObject root) { fill_response_create(root); }));
  if (this->phase_ == Phase::LISTENING)
    this->set_phase_(Phase::THINKING);
}

// =================================================================================================
// Main loop
// =================================================================================================

void OpenAIRealtime::loop() {
  if (this->state_.load() == SessionState::IDLE)
    return;

  if (this->barge_in_pending_.exchange(false))
    this->handle_barge_in_();
  this->process_events_();
  this->update_mic_gate_();

  // The worker parks itself when done; it is deleted here (the stack is kept for the next session).
  if (this->worker_done_.load() && this->worker_.destroy()) {
    this->finish_session_();
    return;
  }
  if (this->state_.load() == SessionState::ACTIVE)
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
      ESP_LOGI(TAG, "Session ready (%s protocol%s)", this->get_flavor_name(),
               ev.a == "assumed" ? ", settings not confirmed by the server" : "");
      this->ready_trigger_.trigger();
      this->set_phase_(Phase::LISTENING);
      break;

    case Event::SPEECH_STARTED:
      this->last_activity_ms_ = now;
      // Turn-based: speech the server heard in audio sent just before the mic closed must not
      // reopen it mid-answer; it is the model's turn until the answer has played.
      if (!this->session_full_duplex_.load() && (this->phase_ == Phase::THINKING || this->phase_ == Phase::SPEAKING))
        break;
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
        // playing) - or the server recovers it via its own TTS right after this response's done.
        // Either way the session closes once everything has drained and stayed quiet (the
        // debounce in check_timeouts_ covers an utterance starting just after the done).
        this->enqueue_(
            json::build_json([&ev](JsonObject root) { fill_function_output(root, ev.c, R"({"ok":true})"); }));
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
      if (!this->ready_fired_) {
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
  // Start an answer only with a little audio in hand, unless that is all there will be.
  if (this->phase_ != Phase::SPEAKING && this->play_off_ >= this->play_len_ && this->response_active_ &&
      this->playback_ring_->available() < PREBUFFER_BYTES &&
      millis() - this->last_response_ms_.load() < RESPONSE_SILENCE_MS)
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
    this->drain_trace_ = 0;  // sync trace: audio flowing again, re-arm the milestones
    if (this->phase_ != Phase::SPEAKING)
      this->set_phase_(Phase::SPEAKING);
    return;
  }

  // Sync trace (turn-based LED lag): one-shot milestones of the drain chain —
  // local ring empty → speaker chain (resampler/mixer/DMA) empty → the
  // response_finished trigger the LED animations hook.  The last delta is the
  // number to compare against the server's "Bot stopped speaking" timestamp.
  if (this->phase_ == Phase::SPEAKING) {
    const bool local_empty = this->playback_ring_->available() == 0 && this->play_off_ >= this->play_len_;
    if (!local_empty) {
      this->drain_trace_ = 0;
    } else if (this->drain_trace_ == 0) {
      this->drain_trace_ = 1;
      ESP_LOGD(TAG, "Drain: local buffers empty (t=%u ms)", millis());
    }
    if (local_empty && !this->speaker_->has_buffered_data() && this->drain_trace_ < 2) {
      this->drain_trace_ = 2;
      ESP_LOGD(TAG, "Drain: speaker chain empty -> response_finished (%u ms after last server response activity)",
               millis() - this->last_response_ms_.load());
    }
  }

  // Drained: nothing buffered here, nothing in the speaker chain, and the server is done talking.
  if (this->phase_ != Phase::SPEAKING || this->playback_ring_->available() != 0 || this->play_off_ < this->play_len_ ||
      this->speaker_->has_buffered_data())
    return;
  if (this->response_active_) {
    // Normally the answer ends with response.done; a server that never sends it is given up on
    // after a quiet spell, so the session does not hang in "speaking".
    if (millis() - this->last_response_ms_.load() < RESPONSE_SILENCE_MS)
      return;
    ESP_LOGW(TAG, "No response.done from the server - taking the answer as finished");
    this->response_active_ = false;
    this->ws_response_active_.store(false);
  }
  this->playing_audio_.store(false);
  this->speaker_->finish();
  this->last_activity_ms_ = millis();
  this->response_finished_trigger_.trigger();
  this->set_phase_(this->pending_calls_ > 0 || this->need_response_create_ ? Phase::THINKING : Phase::LISTENING);
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

void OpenAIRealtime::update_mic_gate_() {
  if (this->end_after_playback_) {
    // The conversation is over by the assistant's call: nothing said from here on can matter -
    // keep the microphone closed (both modes) so no dead audio streams upstream during the end
    // debounce window.
    this->mic_open_.store(false);
    return;
  }
  if (this->session_full_duplex_.load()) {
    this->mic_open_.store(true);
    return;
  }
  // Turn-based: the person's turn is while connecting (the pre-roll is kept), listening and
  // speaking; thinking and the answer are the model's. NONE covers the connect, before the first
  // LISTENING: the worker holds audio back until the session is configured anyway.
  const bool persons_turn =
      this->phase_ == Phase::NONE || this->phase_ == Phase::LISTENING || this->phase_ == Phase::USER_SPEAKING;
  this->mic_open_.store(persons_turn && static_cast<int32_t>(millis() - this->mic_reopen_at_) >= 0);
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
  if (st != SessionState::ACTIVE)
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
  // A response the server never closed (no response.done) and has said nothing about for a long
  // time would otherwise block the next response.create and the idle timeout for good.
  if (this->response_active_ && this->phase_ != Phase::SPEAKING && !this->playing_audio_.load() &&
      now - this->last_response_ms_.load() > 3 * RESPONSE_SILENCE_MS) {
    ESP_LOGW(TAG, "A response never finished - clearing it");
    this->response_active_ = false;
    this->ws_response_active_.store(false);
  }
  const bool quiet = !this->response_active_ && this->pending_calls_ == 0 && !this->need_response_create_ &&
                     !this->playing_audio_.load() && this->playback_ring_->available() == 0 &&
                     this->play_off_ >= this->play_len_;
  if (this->phase_ == Phase::THINKING && quiet && now - this->phase_since_ms_ > THINKING_TIMEOUT_MS) {
    ESP_LOGD(TAG, "No answer to that turn - listening again");
    this->set_phase_(Phase::LISTENING);
    return;
  }
  if (this->end_after_playback_ && quiet && this->phase_ != Phase::SPEAKING) {
    // The end holds only once the quiet has PERSISTED: a farewell the server starts speaking
    // right after the tool-call response's done (its own TTS recovery) arrives as a fresh
    // response within milliseconds - that resets the window instead of ending mid-goodbye.
    if (this->end_quiet_since_ == 0) {
      this->end_quiet_since_ = now;
    } else if (now - this->end_quiet_since_ >= END_QUIET_DEBOUNCE_MS) {
      ESP_LOGI(TAG, "Conversation ended by the assistant");
      this->request_stop();
    }
    return;
  }
  this->end_quiet_since_ = 0;
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
  vTaskSuspend(nullptr);  // the loop deletes the task (StaticTask::destroy)
}

void OpenAIRealtime::run_worker_() {
  esp_websocket_client_config_t cfg{};
  cfg.uri = this->ws_url_.c_str();
  cfg.task_stack = 6144;  // TLS handshake runs on the client's own task
  cfg.task_prio = 5;
  // The client mallocs one receive and one send buffer of this size from internal RAM, which is
  // scarce here (heap allocations do not spill into PSRAM on this board). 4 KB holds a 60 ms mic
  // frame; larger messages (session.update) leave as continuation frames, and larger incoming ones
  // arrive in pieces that ws_event_handler_ reassembles in PSRAM.
  cfg.buffer_size = WS_BUFFER_SIZE;
  cfg.network_timeout_ms = 10000;
  cfg.disable_auto_reconnect = true;
  cfg.reconnect_timeout_ms = 10000;
  cfg.ping_interval_sec = 20;
  cfg.pingpong_timeout_sec = 30;
  if (this->ws_url_.rfind("wss://", 0) == 0) {
    if (!this->ca_certificate_.empty()) {
      // A server with its own (e.g. self-signed) certificate: trust exactly that one.
      cfg.cert_pem = this->ca_certificate_.c_str();
      cfg.cert_len = this->ca_certificate_.size() + 1;  // PEM length includes the terminator
      cfg.skip_cert_common_name_check = !this->verify_certificate_name_;
    } else {
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
      cfg.crt_bundle_attach = esp_crt_bundle_attach;
#endif
    }
  }

  this->client_ = esp_websocket_client_init(&cfg);
  if (this->client_ == nullptr) {
    ESP_LOGE(TAG, "WebSocket client: no memory (internal RAM %u free, largest block %u)",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
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
        if (!this->session_configured_.load()) {
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
  const int r =
      esp_websocket_client_send_text(this->client_, data, static_cast<int>(len), pdMS_TO_TICKS(SEND_TIMEOUT_MS));
  if (r < 0 || static_cast<size_t>(r) != len) {
    if (!this->worker_stop_.load())
      this->push_event_({Event::TRANSPORT_ERROR, "send", "Sending to the Realtime server failed"});
    return false;
  }
  return true;
}

bool OpenAIRealtime::send_mic_audio_() {
  // Whole 60 ms chunks only; until one is there, nap briefly (this loop paces itself on the mic).
  if (this->mic_ring_->available() < MIC_CHUNK) {
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
  }
  size_t n = this->mic_ring_->read(this->pcm_buf_, MIC_CHUNK, 0);
  n &= ~static_cast<size_t>(1);
  if (n == 0)
    return true;
  if (!this->mic_open_.load())
    return true;  // turn-based: not the person's turn - drop it, so it is never sent late

  // The one message not built with ArduinoJson (see rt_messages.h): base64 straight into the
  // reused frame buffer.
  const size_t total = build_audio_append(this->pcm_buf_, n, this->tx_buf_, this->tx_cap_);
  if (total == 0)
    return true;
  const uint32_t t0 = millis();
  if (!this->send_raw_(this->tx_buf_, total))
    return false;
  const uint32_t took = millis() - t0;
  if (took >= SEND_STALL_MS) {
    // The server stopped reading for a while (its TCP window closed). What the microphone queued
    // meanwhile would reach it seconds late - as if just spoken - so it is dropped.
    ESP_LOGW(TAG, "The server did not read for %" PRIu32 " ms; dropping %u bytes of stale microphone audio", took,
             static_cast<unsigned>(this->mic_ring_->available()));
    this->mic_ring_->reset();
  }
  return true;
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
  if (this->playback_ring_ == nullptr)
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

void OpenAIRealtime::note_response_event_(const std::string &type) {
  this->last_response_ms_.store(millis());
  if (type == "response.created" || type == "response.done")
    return;
  // Any other response.* event means an answer is under way. Some servers never send
  // response.created; without this the device would take the first gap in the audio for the end of
  // the answer (stop the speaker, and in turn-based mode reopen the microphone mid-answer).
  if (!this->ws_response_active_.exchange(true))
    this->push_event_({Event::RESPONSE_CREATED, "implicit", ""});
}

void OpenAIRealtime::on_ws_message_(char *buf, size_t len) {
  const std::string type = first_type(buf, len);
  if (type.rfind("response.", 0) == 0)
    this->note_response_event_(type);
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
      const bool audio_pending = this->playing_audio_.load() || this->ws_response_active_.load() ||
                                 (this->playback_ring_ != nullptr && this->playback_ring_->available() > 0);
      // Turn-based sessions have no barge-in: the server does not cancel (interrupt_response is
      // off) and the answer plays out; only the wake word or the button stops it.
      if (this->session_full_duplex_.load() && !this->current_item_.empty() && audio_pending &&
          this->current_item_ != this->dropped_item_) {
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

}  // namespace openai_realtime
}  // namespace esphome

#endif  // USE_ESP32
