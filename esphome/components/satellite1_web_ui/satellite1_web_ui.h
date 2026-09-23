#pragma once

#include <atomic>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include "session_gate.h"
#include "web_ui_handler.h"

#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#ifdef USE_SAT1_WEB_UI_SENDSPIN
#include "esphome/components/sendspin/sendspin_hub.h"
#endif

namespace esphome {
namespace satellite1_web_ui {

/// Appends `src` to `out` with the whitespace a readable YAML source carries stripped out: Jinja
/// `{# ... #}` comments are dropped and every run of whitespace collapses to one space.
///
/// Exists because the response templates in common/web_ui_ha.yaml and common/web_ui_media.yaml are
/// re-sent to Home Assistant on every page call, and the API server's shared write buffer grows to
/// the largest message it ever encodes and never shrinks - so every byte of indentation in the
/// template is pinned on the internal heap for the life of the connection, plus paid again in the
/// transient encode copy per call. Measured on the HA template: 9,937 bytes readable, 7,673
/// minified.
///
/// Safe because of two properties of these templates, both checked by the caller's review rather
/// than at runtime: no Jinja string literal in them contains consecutive whitespace, a tab or a
/// newline (single spaces survive - only *runs* collapse), and no literal contains `{#`. Whitespace
/// between tags is rendered output, but the only output these templates produce outside their final
/// `{{ ... }}` expression is whitespace, which literal_eval on the Home Assistant side ignores.
///
/// A template over the string type so the PSRAM-backed basic_string the YAML globals use needs no
/// conversion through an internal-heap std::string.
template<typename S> void append_minified_jinja(S &out, const char *src, size_t len) {
  bool pending_space = false;
  for (size_t i = 0; i < len; i++) {
    const char c = src[i];
    if (c == '{' && i + 1 < len && src[i + 1] == '#') {
      size_t j = i + 2;
      while (j + 1 < len && !(src[j] == '#' && src[j + 1] == '}'))
        j++;
      i = j + 1;  // Lands on the closing '}'; the loop's increment steps past it.
      pending_space = true;
      continue;
    }
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
      pending_space = true;
      continue;
    }
    if (pending_space && !out.empty())
      out.push_back(' ');
    pending_space = false;
    out.push_back(c);
  }
}

/**
 * Serves the on-device web app from PROGMEM as a handler on ESPHome's shared web server.
 *
 * The whole component exists at setup_priority::WIFI (250) for one reason: registration order is
 * what decides who answers "/". WebServerBase::add_handler appends to a vector that
 * AsyncWebServer::request_handler_ walks in order, first canHandle wins, and WebServer claims "/"
 * unconditionally at web_server.cpp:2339. WebServer::get_setup_priority() is WIFI - 1.0f = 249, so
 * anything above that registers first and takes the root.
 *
 * Registering from setup() rather than from codegen is load bearing, not stylistic.
 * WebServerBase::add_handler wraps a handler in AuthMiddlewareHandler only if credentials_.is_set()
 * at the moment it is called, and web_server's to_code emits set_auth_username() as a statement in
 * main.cpp. Every codegen statement runs before App.setup(), so a setup() registration is always
 * after the username is installed. A codegen registration placed ahead of web_server's own
 * statement would put the whole app on port 80 unauthenticated, silently.
 */
class Satellite1WebUI : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::WIFI; }

  void set_index(const uint8_t *gz, size_t gz_len) { this->handler_.set_index(gz, gz_len); }
  void set_etag(const char *etag) { this->handler_.set_etag(etag); }
  void set_no_sensor_image(const uint8_t *webp, size_t len) { this->handler_.set_no_sensor_image(webp, len); }
  void set_manifest(const uint8_t *json, size_t len) { this->handler_.set_manifest(json, len); }
  void set_icon(uint8_t which, const uint8_t *png, size_t len) { this->handler_.set_icon(which, png, len); }
  void add_entity(const char *key, const char *domain, EntityBase *entity) {
    this->handler_.add_entity(key, domain, entity);
  }

#ifdef USE_SAT1_WEB_UI_SOUNDS
  /// The sounds served at /api/sat1/sounds/<name>, from generated code like the entity table.
  void add_sound(const char *name, const audio::AudioFile *file) { this->handler_.add_sound(name, file); }
#endif

  /* ---- The session gate: cookie auth and the device-presence login. ---- */

  /// Called from generated code in main.cpp, before any component's setup() - which is what puts
  /// the gate at position 0 of web_server_base's handler vector, ahead of satellite1_radar (which
  /// registers at setup priority 800) and everything after. Registered without auth wrapping on
  /// purpose: the gate *is* the auth, and wrapping it in AuthMiddlewareHandler would put a digest
  /// prompt in front of the login page itself.
  void register_session_gate() { web_server_base::global_web_server_base->add_handler_without_auth(&this->gate_); }

  /// The credentials, handed over by the on_boot lambda in common/web_ui.yaml at priority 600 -
  /// after the stored password is restored, before the gate computes its token at setup (250).
  /// These no longer go to web_server_base at all, so no handler is ever wrapped in the digest
  /// middleware; the gate runs the same digest check itself as its curl/script fallback.
  void set_credentials(const char *username, const std::string &password) {
    this->gate_.set_credentials(username, password);
  }

  /// Whether a voice approval could be heard right now - the mute slider and switch, read at
  /// window-open time. From YAML because only YAML knows which entities mean "muted" on this build.
  void set_login_mic_available(std::function<bool()> fn) { this->gate_.set_mic_available_fn(std::move(fn)); }

#ifdef USE_SWITCH
  /// The software mute switch the held-mute endpoint drives (mute_switch: in web_ui.yaml -
  /// master_mute_switch on this product). From generated code; without it POST /api/sat1/mutehold
  /// answers 404 and the tune-time peer muting simply does not exist on this build.
  void set_mute_switch(switch_::Switch *sw) { this->mute_switch_ = sw; }
#endif

  /// The action button's approval. Returns true when a pending window was approved, which is the
  /// dispatcher's cue to consume the press instead of running its normal single-press action.
  bool approve_pending_login() { return this->gate_.approve_pending_login(); }

  /// The voice approvals, called from the on_stt_end and on_wake_word_detected hooks in
  /// common/voice_assistant.yaml. True means consumed: suppress the transcript ring / skip the
  /// assistant start, because the utterance was an answer to the pairing window.
  bool consume_login_transcript(const std::string &text) { return this->gate_.consume_login_transcript(text); }
  bool consume_login_wake(const std::string &phrase) { return this->gate_.consume_login_wake(phrase); }

  /// Window state reads for the YAML scripts: whether any window is pending (the LED breathe), and
  /// whether an online-mode window is still waiting for a code (the re-listen loop).
  bool login_pending() { return this->gate_.login_pending(); }
  bool login_voice_pending() { return this->gate_.login_voice_pending(); }

  /// Fired from loop() when a pairing window opens, with the mode ("button"/"code"/"seq") and the
  /// secret ("4271", or the challenge sequence as symbol digits "012" for hey_jarvis/okay_nabu/
  /// stop). YAML owns the announcement and the LED from here.
  Trigger<std::string, std::string> *get_login_window_trigger() { return &this->login_window_trigger_; }

  /// Fired from loop() when the window closes, with the result ("approved"/"expired"/"denied").
  Trigger<std::string> *get_login_window_end_trigger() { return &this->login_window_end_trigger_; }

#ifdef USE_VOICE_ASSISTANT
  void set_voice_assistant(voice_assistant::VoiceAssistant *va) { this->handler_.set_voice_assistant(va); }
  void set_voice_phase_fn(std::function<int()> fn) { this->handler_.set_voice_phase_fn(std::move(fn)); }

  /// Called from the on_stt_end and on_tts_start lambdas in common/voice_assistant.yaml.
  void push_utterance(const std::string &text, bool heard) { this->handler_.push_utterance(text, heard); }
#endif

#ifdef USE_MICRO_WAKE_WORD
  void set_micro_wake_word(micro_wake_word::MicroWakeWord *mww) {
    this->handler_.set_micro_wake_word(mww);
    // The offline wake-word challenge exists exactly when the models it is built from do.
    this->gate_.set_seq_available(mww != nullptr);
  }

  /// Called from the on_wake_word_detected automation in common/voice_assistant.yaml - after the
  /// sign-in gate has had first refusal, so a challenge answer never lands in a readable history.
  void push_wake_detection(const std::string &word) { this->handler_.push_wake_detection(word); }

  /// Whether the app has a "say it now" test window open - the automation then records the firing
  /// (above) but skips the chime and the assistant start. See WebUIHandler::wake_test_active.
  bool wake_test_active() { return this->handler_.wake_test_active(); }
#endif

#ifdef USE_SAT1_MWW_LOADER
  void set_wake_loader(mww_runtime_loader::MwwRuntimeLoader *loader) { this->handler_.set_wake_loader(loader); }
#endif

#ifdef USE_SAT1_CRASH_REPORT
  /// The crash flight recorder, for the /api/sat1/crash* endpoints. From generated code, so it is
  /// set long before the listener accepts anything.
  void set_crash_report(crash_report::CrashReport *cr) { this->handler_.set_crash_report(cr); }
#endif

#ifdef USE_MEDIA_PLAYER
  void set_media_player(media_player::MediaPlayer *mp) { this->handler_.set_media_player(mp); }
  void set_sendspin_media_player(media_player::MediaPlayer *mp) { this->handler_.set_sendspin_media_player(mp); }
#endif

#ifdef USE_SAT1_WEB_UI_SENDSPIN
  /// The hub itself, beyond the media_player entity above, for what the entity model cannot say:
  /// track metadata with an artwork URL, controller state with shuffle/repeat and the server's
  /// supported-command list, and the interpolated track position. setup() subscribes to the hub's
  /// callbacks - all of which fire on the main loop - and translates them into the handler's plain
  /// setters, so the handler never includes a sendspin header.
  void set_sendspin_hub(sendspin_::SendspinHub *hub) { this->sendspin_hub_ = hub; }
#endif

  /// All called from the rung lambdas in common/web_ui_ha.yaml.
  ///
  /// The payload arrives as a buffer rather than a string, in two steps: stage hands out PSRAM to write
  /// into, commit publishes what was written. What that buys is that the several kilobytes Home Assistant
  /// renders never exist on the internal heap - a std::string of them did, and the allocation aborted the
  /// device on syncs where the heap could not find one contiguous block that size.
  ///
  /// Commit also lifts `aid` out of the payload into the selection store, which is the only moment the
  /// device can learn its own Home Assistant area. Done here rather than in the YAML lambda so the one
  /// place that receives the payload is the one place that unpacks it.
  char *stage_ha_payload(size_t capacity);
  void commit_ha_payload(size_t len, int rung);

  /// The paged form of the pair above, for the ladder in common/web_ui_ha.yaml: one begin, a
  /// stage_ha_page per reply, one commit. See WebUIHandler::begin_ha_pages for why it is paged.
  void begin_ha_pages() { this->handler_.begin_ha_pages(); }
  char *stage_ha_page(size_t len) { return this->handler_.stage_ha_page(len); }
  void commit_ha_pages(int rung);

  /// The same thing for a caller that already holds the payload as a string. Not what the sync uses: by
  /// the time such a string exists, the internal-heap copy the pair above avoids has been paid for.
  void set_ha_payload(const std::string &json, int rung);
  void set_ha_failed() { this->handler_.set_ha_failed(); }

  /// The actions-checkbox verdict from tts_routing's probe (0 unknown, 1 allowed, 2 blocked,
  /// 3 Home Assistant too old), pushed from tts_routing_status_publish and served as `actions`
  /// on /api/sat1/ha. See WebUIHandler::set_ha_actions.
  void set_ha_actions(int verdict) { this->handler_.set_ha_actions(verdict); }

  /// Fired from loop() when a browser has posted to /api/sat1/ha/refresh. A trigger rather than a
  /// direct call, because the work is a Home Assistant action and the script that owns it lives in
  /// YAML - which also keeps the ladder's only caller in one file.
  Trigger<> *get_ha_refresh_trigger() { return &this->ha_refresh_trigger_; }

  /// Fired from loop() with an entity id and an option, for the Home Assistant selects that decide
  /// which assistant answers which wake word. The action call itself is in YAML for the same reason the
  /// refresh ladder is: homeassistant.action belongs next to the rest of the data layer.
  ///
  /// At most one per iteration, so the automation behind this is never re-entered while running. A
  /// `delay:` in it would still be a mistake - it would stall the queue rather than corrupt it.
  Trigger<std::string, std::string> *get_ha_select_trigger() { return &this->ha_select_trigger_; }

  /// The Music Assistant relay's triggers, all fired from loop() and all implemented in
  /// common/web_ui_media.yaml, one homeassistant.action each - the same split as the pair above.
  /// The refresh is the sync script; the rest carry the entity a browser named (validated against
  /// the device's own payloads in the endpoint) and the command's one value where it has one.
  Trigger<> *get_ma_refresh_trigger() { return &this->ma_refresh_trigger_; }
  Trigger<std::string> *get_ma_like_trigger() { return &this->ma_like_trigger_; }
  Trigger<std::string, std::string> *get_ma_join_trigger() { return &this->ma_join_trigger_; }
  Trigger<std::string> *get_ma_unjoin_trigger() { return &this->ma_unjoin_trigger_; }
  Trigger<std::string, float> *get_ma_volume_trigger() { return &this->ma_volume_trigger_; }
  Trigger<std::string, float> *get_ma_seek_trigger() { return &this->ma_seek_trigger_; }

  /// The Music Assistant payload's staging trio, called from the sync script's lambdas in
  /// common/web_ui_media.yaml - the HA trio's shape, same PSRAM discipline. Paged because the
  /// members payload is bounded by how many speakers one home can group rather than by page size,
  /// and one oversized reply would re-pin the API's receive buffer at that size for the rest of the
  /// connection - the exact cost the HA paging exists to avoid.
  void begin_ma_pages() { this->handler_.begin_ma_pages(); }
  char *stage_ma_page(size_t len) { return this->handler_.stage_ma_page(len); }
  void commit_ma_pages() { this->handler_.commit_ma_pages(); }

  /// Fired from loop() after the app has written a new selection. tts_routing.yaml hangs its re-check
  /// scripts here, in place of the `on_value` the deleted Remote TTS Targets text entity carried.
  ///
  /// Deferred to loop() rather than fired from the change callback, because that callback runs on the
  /// httpd task and the scripts it starts perform Home Assistant actions.
  Trigger<> *get_selection_change_trigger() { return &this->selection_change_trigger_; }

  /// What the app has chosen, and the only copy of it. Read from YAML lambdas in tts_routing.yaml and
  /// area_ducking.yaml, and by the template switches that project it.
  Selection &selection() { return this->selection_; }

 protected:
  /// Takes this device's own area id out of a freshly committed payload. Null is the refused-commit
  /// case and does nothing.
  void adopt_ha_area_(const char *json);

  WebUIHandler handler_;
  SessionGate gate_;
  Selection selection_;
  Trigger<std::string, std::string> login_window_trigger_;
  Trigger<std::string> login_window_end_trigger_;
#ifdef USE_SAT1_WEB_UI_SENDSPIN
  sendspin_::SendspinHub *sendspin_hub_{nullptr};
#endif
  Trigger<> ha_refresh_trigger_;
  Trigger<std::string, std::string> ha_select_trigger_;
  Trigger<> ma_refresh_trigger_;
  Trigger<std::string> ma_like_trigger_;
  Trigger<std::string, std::string> ma_join_trigger_;
  Trigger<std::string> ma_unjoin_trigger_;
  Trigger<std::string, float> ma_volume_trigger_;
  Trigger<std::string, float> ma_seek_trigger_;
  Trigger<> selection_change_trigger_;
  std::atomic<bool> selection_changed_{false};

#ifdef USE_SWITCH
  /* ---- The tune-time held mute (POST /api/sat1/mutehold; serviced in loop()) ----
   * A peer's tuner holds this device muted while its session runs. The hold is this device's own
   * state, deliberately: it remembers the prior mute, restores it on release OR on the TTL below
   * expiring, and never trusts the browser that asked to come back - the exact self-healing shape
   * the tune session itself uses. millis_64() for the deadline, like every deadline here. */
  switch_::Switch *mute_switch_{nullptr};
  bool mute_hold_{false};
  bool mute_hold_prior_on_{false};
  /// Set once the switch has been OBSERVED on while held. The manual-unmute-wins rule (a person at
  /// the device outranks a remote tune session; owner decision, September 23 2026) reads "held but
  /// off" as that person's hand - but only after this flag, because turn_on() through the template
  /// switch is refused while the hardware mute slider is engaged, and treating that unhonored
  /// request as a manual unmute would cancel the hold it never applied.
  bool mute_hold_seen_on_{false};
  uint64_t mute_hold_until_{0};
#endif

  /// The last time the MA refresh trigger fired, for the floor loop() applies: browsers refresh on a
  /// cadence of their own while the footer is expanded, several tabs can do it at once, and each sync
  /// is an action call - so requests inside the window ride the sync already in flight.
  uint32_t ma_refresh_at_{0};

  /// Our loop() runs once per main-loop iteration, so the gap between two calls is the main loop
  /// period. That makes the loop-time readout free, where the debug: component would cost a sensor
  /// entity riding /events to every open tab for a number only Diagnostics ever shows.
  uint32_t last_loop_ms_{0};
  std::atomic<uint32_t> max_loop_ms_{0};
};

}  // namespace satellite1_web_ui
}  // namespace esphome
