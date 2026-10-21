#pragma once

#ifdef USE_ESP32

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_event.h>
#include <esp_websocket_client.h>

#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/components/speaker/speaker.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/static_task.h"

#include "rt_messages.h"
#include "rt_util.h"

namespace esphome {
namespace openai_realtime {

/// What the device is doing with a session. Exposed to YAML (openai_realtime.is_active).
enum class SessionState : uint8_t {
  IDLE = 0,    ///< no session
  CONNECTING,  ///< WebSocket/TLS handshake running
  ACTIVE,      ///< session configured, audio flowing both ways
  CLOSING,     ///< teardown requested, worker closing the socket
};

/// The conversational phase inside an ACTIVE session - what the LED ring shows.
enum class Phase : uint8_t {
  NONE = 0,
  LISTENING,      ///< waiting for the person to speak
  USER_SPEAKING,  ///< server VAD heard speech start
  THINKING,       ///< speech ended, response not audible yet
  SPEAKING,       ///< model audio is playing
};

const char *session_state_name(SessionState s);
const char *phase_name(Phase p);

/// A server event, digested on the WebSocket task and handed to the main loop.
struct Event {
  enum Kind : uint8_t {
    CONNECTED,
    SESSION_READY,
    SPEECH_STARTED,
    SPEECH_STOPPED,
    RESPONSE_CREATED,
    RESPONSE_DONE,
    FUNCTION_CALL,
    USER_TRANSCRIPT,
    ASSISTANT_TRANSCRIPT,
    SERVER_ERROR,
    TRANSPORT_ERROR,
  } kind;
  std::string a;    ///< name / transcript / code
  std::string b{};  ///< arguments / message
  std::string c{};  ///< call_id
};

class OpenAIRealtime : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ---- codegen setters (all configuration comes from YAML) ----
  void set_base_url(const std::string &v) { this->base_url_ = v; }
  void set_model(const std::string &v) { this->model_ = v; }
  void set_voice(const std::string &v) { this->voice_ = v; }
  void set_api_key(const std::string &v) { this->api_key_ = v; }
  void set_instructions(const std::string &v) { this->instructions_ = v; }
  void set_turn_detection(const std::string &type, const std::string &eagerness, float threshold,
                          uint32_t prefix_padding_ms, uint32_t silence_duration_ms) {
    this->td_type_ = type;
    this->td_eagerness_ = eagerness;
    this->td_threshold_ = threshold;
    this->td_prefix_ms_ = prefix_padding_ms;
    this->td_silence_ms_ = silence_duration_ms;
  }
  void set_noise_reduction(const std::string &v) { this->noise_reduction_ = v; }
  void set_input_transcription_model(const std::string &v) { this->transcription_model_ = v; }
  void set_tools_json(const std::string &v) { this->tools_json_ = v; }
  void set_end_conversation_tool(bool v) { this->end_tool_ = v; }
  /// Full duplex (true): the microphone stays open while the model speaks, and talking over it
  /// interrupts it. Turn-based (false): like the Assist pipeline - the microphone closes when the
  /// person stops talking and reopens after the answer has played; only the wake word interrupts.
  /// Callable at any time (a switch); a conversation keeps the mode it started with.
  void set_full_duplex(bool v) { this->full_duplex_.store(v); }
  bool is_full_duplex() const { return this->full_duplex_.load(); }
  /// "auto" (detect from session.created), "ga" or "beta".
  void set_api_version(const std::string &v) {
    this->forced_flavor_ = v == "ga" ? Flavor::GA : (v == "beta" ? Flavor::BETA : Flavor::UNKNOWN);
  }
  void set_idle_timeout(uint32_t ms) { this->idle_timeout_ms_ = ms; }
  void set_max_duration(uint32_t ms) { this->max_duration_ms_ = ms; }
  void set_playback_latency(uint32_t ms) { this->playback_latency_ms_ = ms; }
  void set_playback_buffer_size(size_t n) { this->playback_buffer_size_ = n; }
  void set_extra_header(const std::string &k, const std::string &v) { this->extra_headers_.emplace_back(k, v); }
  /// PEM certificate (or CA) that a wss:// server must present instead of a publicly trusted one.
  void set_ca_certificate(const std::string &pem) { this->ca_certificate_ = pem; }
  /// false: accept that certificate whatever host name or IP address it was issued for.
  void set_verify_certificate_name(bool v) { this->verify_certificate_name_ = v; }
  void set_microphone_source(microphone::MicrophoneSource *m) { this->mic_source_ = m; }
  void set_speaker(speaker::Speaker *s) { this->speaker_ = s; }
  void set_muted_lambda(std::function<bool()> &&f) { this->muted_fn_ = std::move(f); }
  void set_has_function_handler(bool v) { this->has_function_handler_ = v; }

  // ---- runtime control (main loop / automations) ----
  void request_start();
  void request_stop();
  /// Answers a model's function call. `respond` asks the model to continue speaking afterwards.
  void send_function_result(const std::string &call_id, const std::string &output, bool respond = true);
  /// Adds a user text message and asks for a spoken answer (handy for announcements and tests).
  void send_text(const std::string &text);

  bool is_active() const { return this->state_.load() != SessionState::IDLE; }
  bool is_configured() const;
  SessionState get_state() const { return this->state_.load(); }
  /// "ga", "beta" or "" - the wire format the current or last session used.
  const char *get_flavor_name() const;
  Phase get_phase() const { return this->phase_; }

  // ---- triggers ----
  Trigger<> *get_start_trigger() { return &this->start_trigger_; }
  Trigger<> *get_ready_trigger() { return &this->ready_trigger_; }
  Trigger<> *get_listening_trigger() { return &this->listening_trigger_; }
  Trigger<> *get_speech_started_trigger() { return &this->speech_started_trigger_; }
  Trigger<> *get_speech_stopped_trigger() { return &this->speech_stopped_trigger_; }
  Trigger<> *get_response_started_trigger() { return &this->response_started_trigger_; }
  Trigger<> *get_response_finished_trigger() { return &this->response_finished_trigger_; }
  Trigger<> *get_interrupted_trigger() { return &this->interrupted_trigger_; }
  Trigger<> *get_end_trigger() { return &this->end_trigger_; }
  Trigger<std::string, std::string> *get_error_trigger() { return &this->error_trigger_; }
  Trigger<std::string> *get_user_transcript_trigger() { return &this->user_transcript_trigger_; }
  Trigger<std::string> *get_assistant_transcript_trigger() { return &this->assistant_transcript_trigger_; }
  Trigger<std::string, std::string, std::string> *get_function_call_trigger() { return &this->function_call_trigger_; }

 protected:
  // ---- session lifecycle (main loop) ----
  void start_session_();
  void finish_session_();
  void set_phase_(Phase p);
  void build_session_update_();
  void release_buffers_();
  void enqueue_(std::string &&msg);
  void process_events_();
  void handle_event_(Event &ev);
  void feed_speaker_();
  void handle_barge_in_();
  void check_timeouts_();
  void update_mic_gate_();
  void fail_(const std::string &code, const std::string &message);
  void maybe_request_response_();

  // ---- worker task: owns the socket ----
  static void worker_task_(void *arg);
  void run_worker_();
  bool send_raw_(const char *data, size_t len);
  bool send_mic_audio_();

  // ---- WebSocket task callbacks ----
  static void ws_event_handler_(void *arg, esp_event_base_t base, int32_t id, void *data);
  void on_ws_data_(const esp_websocket_event_data_t *d);
  void on_ws_message_(char *buf, size_t len);
  void note_response_event_(const std::string &type);
  void on_audio_delta_(char *buf, size_t len);
  void push_event_(Event &&ev);

  // ---- microphone task callback ----
  void on_mic_data_(const std::vector<uint8_t> &data);

  // config
  std::string base_url_;
  std::string model_;
  std::string voice_;
  std::string api_key_;
  std::string instructions_;
  std::string td_type_{"semantic_vad"};
  std::string td_eagerness_{"auto"};
  float td_threshold_{0.5f};
  uint32_t td_prefix_ms_{300};
  uint32_t td_silence_ms_{500};
  std::string noise_reduction_;
  std::string transcription_model_;
  std::string tools_json_;
  bool end_tool_{true};
  bool has_function_handler_{false};
  std::atomic<bool> full_duplex_{true};  // the setting; read at each session start
  uint32_t idle_timeout_ms_{20000};
  uint32_t max_duration_ms_{15 * 60 * 1000};
  uint32_t playback_latency_ms_{600};
  size_t playback_buffer_size_{768 * 1024};
  std::vector<std::pair<std::string, std::string>> extra_headers_;
  std::string ca_certificate_;
  bool verify_certificate_name_{true};
  microphone::MicrophoneSource *mic_source_{nullptr};
  speaker::Speaker *speaker_{nullptr};
  std::function<bool()> muted_fn_;

  // session (main loop owns unless noted)
  std::atomic<SessionState> state_{SessionState::IDLE};
  Phase phase_{Phase::NONE};
  bool stop_requested_{false};
  uint32_t session_started_ms_{0};
  uint32_t last_activity_ms_{0};
  bool response_active_{false};
  bool need_response_create_{false};
  int pending_calls_{0};
  bool end_after_playback_{false};
  uint32_t end_quiet_since_{0};
  bool ready_fired_{false};
  std::string session_update_;
  std::string ws_url_;
  std::string auth_header_;
  HighFrequencyLoopRequester high_freq_;

  // worker <-> loop
  StaticTask worker_;  // stack in PSRAM, kept between sessions
  std::atomic<bool> worker_stop_{false};
  std::atomic<bool> worker_done_{false};
  std::atomic<bool> ws_connected_{false};
  std::atomic<bool> session_configured_{false};
  std::atomic<bool> session_created_{false};
  std::atomic<uint8_t> flavor_{0};  // Flavor of the live (or last) session
  Flavor forced_flavor_{Flavor::UNKNOWN};
  std::string session_update_beta_;
  std::atomic<bool> transport_failed_{false};
  esp_websocket_client_handle_t client_{nullptr};
  Mutex out_lock_;
  std::deque<std::string> out_queue_;

  // ws task -> loop
  Mutex event_lock_;
  std::deque<Event> events_;
  std::string close_reason_;  // ws task writes before DISCONNECTED; loop reads after the worker ends

  // inbound frame assembly (ws task only)
  char *rx_buf_{nullptr};
  size_t rx_cap_{0};
  size_t rx_len_{0};
  bool rx_overflow_{false};

  // audio
  std::unique_ptr<ring_buffer::RingBuffer> mic_ring_;       // 24 kHz PCM16 mono, mic task -> worker
  std::unique_ptr<ring_buffer::RingBuffer> playback_ring_;  // 24 kHz PCM16 mono, ws task -> loop
  Upsampler2to3 upsampler_;
  std::vector<int16_t> up_buf_;
  char *tx_buf_{nullptr};
  size_t tx_cap_{0};
  uint8_t *pcm_buf_{nullptr};
  uint8_t *play_buf_{nullptr};
  size_t play_len_{0};
  size_t play_off_{0};
  std::atomic<bool> playing_audio_{false};
  // The duplex mode this session started with (set by the loop before the worker starts; read by
  // the WebSocket task for barge-in) and, in turn-based mode, whether mic audio is sent now.
  std::atomic<bool> session_full_duplex_{true};
  std::atomic<bool> mic_open_{true};  // loop writes, worker reads
  uint32_t mic_reopen_at_{0};         // loop: turn-based, when the mic may reopen after an answer
  uint32_t phase_since_ms_{0};        // loop: when phase_ last changed
  std::atomic<bool> accepting_mic_{false};
  std::atomic<bool> ws_response_active_{false};  // ws task: between response.created and response.done
  std::atomic<uint32_t> last_response_ms_{0};    // ws task: when the last response.* event arrived

  // playback accounting for truncation (bytes, monotonic within a session)
  std::atomic<uint32_t> bytes_written_{0};  // ws task: into playback_ring_
  // Stream position (same coordinates as bytes_written_) up to which audio has left this component:
  // accepted by the speaker, or discarded by a barge-in. Item offsets are in these coordinates, so
  // a discard must advance it too, or every later item's "heard" estimate would be short by it.
  uint32_t bytes_played_{0};
  Mutex item_lock_;
  std::string current_item_;                      // the assistant item whose audio is being written
  std::atomic<uint32_t> current_item_offset_{0};  // bytes_written_ when its first delta arrived
  std::string dropped_item_;                      // an interrupted item; its late deltas are discarded
  std::string barge_item_;                        // the item a barge-in interrupted, for the truncate
  uint32_t barge_offset_{0};
  uint32_t first_play_offset_{UINT32_MAX};  // loop: which item item_first_play_ms_ belongs to
  uint32_t item_first_play_ms_{0};
  std::atomic<bool> barge_in_pending_{false};

  Trigger<> start_trigger_;
  Trigger<> ready_trigger_;
  Trigger<> listening_trigger_;
  Trigger<> speech_started_trigger_;
  Trigger<> speech_stopped_trigger_;
  Trigger<> response_started_trigger_;
  Trigger<> response_finished_trigger_;
  Trigger<> interrupted_trigger_;
  Trigger<> end_trigger_;
  Trigger<std::string, std::string> error_trigger_;
  Trigger<std::string> user_transcript_trigger_;
  Trigger<std::string> assistant_transcript_trigger_;
  Trigger<std::string, std::string, std::string> function_call_trigger_;
};

// ---- actions and conditions ----

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<OpenAIRealtime> {
 public:
  void play(const Ts &...x) override { this->parent_->request_start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<OpenAIRealtime> {
 public:
  void play(const Ts &...x) override { this->parent_->request_stop(); }
};

template<typename... Ts> class FunctionResultAction : public Action<Ts...>, public Parented<OpenAIRealtime> {
  TEMPLATABLE_VALUE(std::string, call_id)
  TEMPLATABLE_VALUE(std::string, output)
  TEMPLATABLE_VALUE(bool, respond)

 public:
  void play(const Ts &...x) override {
    this->parent_->send_function_result(this->call_id_.value(x...), this->output_.value(x...),
                                        this->respond_.has_value() ? this->respond_.value(x...) : true);
  }
};

template<typename... Ts> class SetFullDuplexAction : public Action<Ts...>, public Parented<OpenAIRealtime> {
  TEMPLATABLE_VALUE(bool, full_duplex)

 public:
  void play(const Ts &...x) override { this->parent_->set_full_duplex(this->full_duplex_.value(x...)); }
};

template<typename... Ts> class SendTextAction : public Action<Ts...>, public Parented<OpenAIRealtime> {
  TEMPLATABLE_VALUE(std::string, text)

 public:
  void play(const Ts &...x) override { this->parent_->send_text(this->text_.value(x...)); }
};

template<typename... Ts> class IsActiveCondition : public Condition<Ts...>, public Parented<OpenAIRealtime> {
 public:
  bool check(const Ts &...x) override { return this->parent_->is_active(); }
};

}  // namespace openai_realtime
}  // namespace esphome

#endif  // USE_ESP32
