#pragma once

// Before any USE_* test in this file. Without it the guards below are evaluated against an empty
// macro set - the conditional include gets skipped, and then a transitive defines.h from one of the
// headers underneath turns the same guards on for the member declarations, which then reference a
// type that was never included.
#include "esphome/core/defines.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#ifdef USE_MICRO_WAKE_WORD
#include "esphome/components/micro_wake_word/micro_wake_word.h"
#endif

#ifdef USE_SAT1_MWW_LOADER
#include "esphome/components/mww_runtime_loader/mww_runtime_loader.h"
#endif

#ifdef USE_VOICE_ASSISTANT
#include "esphome/components/voice_assistant/voice_assistant.h"
#endif

#ifdef USE_MEDIA_PLAYER
#include "esphome/components/media_player/media_player.h"
#endif

#ifdef USE_SAT1_WEB_UI_SOUNDS
#include "esphome/components/audio/audio.h"
#endif

#include "esphome/core/entity_base.h"
#include "esphome/core/helpers.h"

#include "esphome/components/web_server_base/web_server_base.h"

#include "selection.h"

namespace esphome {
namespace satellite1_web_ui {

/// Where the app is served. "/" is the real home; "/ui/" is a canary. If our handler ever stops
/// registering ahead of web_server the root goes back to ESPHome's stock dashboard, and the alias
/// is what makes that a cosmetic regression instead of an unreachable app.
static const char *const WU_URL_ROOT = "/";
static const char *const WU_URL_ALIAS = "/ui";
static const char *const WU_URL_ALIAS_SLASH = "/ui/";

/// How many exchanges the Voice card keeps. Eight is about a screen's worth on a phone, and the
/// point is "what did it just mishear", not a conversation history - which would want storage and a
/// retention policy that a device with 165KB of internal heap free has no business offering.
static constexpr size_t WU_TRANSCRIPT_RING = 8;

/// One heard-or-said line. `heard` distinguishes the two so the UI can attribute them without
/// carrying a second string per entry.
struct Utterance {
  std::string text;
  uint32_t at_uptime;
  bool heard;
};

#ifdef USE_MICRO_WAKE_WORD
/// One wake word firing, for the "say it now" test moment and the Diagnostics history. The same
/// ring serves both readers. Held as uptime so the payload can report an honest time-ago whatever
/// the browser's clock thinks.
struct WakeDetection {
  std::string word;
  uint32_t at_ms;
};

/// Eight, like the transcript ring and for the same reason: this is "what fired recently", not a
/// history, and it is never written to flash.
static constexpr size_t WU_DETECTION_RING = 8;
#endif

/// One queued change to a Home Assistant select, waiting for the main loop to turn it into an action
/// call. See the queue itself for why it cannot be made from the request.
struct SelectWrite {
  std::string entity;
  std::string option;
};

/// How many of those can be outstanding. Four is the most one press can produce: choosing an assistant
/// for a wake word may have to move that wake word into a slot and set the slot's pipeline, and doing
/// that for both slots at once is four. A fifth is refused rather than dropped, so the app finds out.
static constexpr size_t WU_SELECT_QUEUE = 4;

/// One queued Music Assistant command, waiting for the main loop to fire the trigger that turns it
/// into a Home Assistant action call - the same deferral as SelectWrite, for the same reason. `arg`
/// is the command's second value where it has one: the entity being joined, a volume, a position.
struct MaWrite {
  uint8_t kind;
  std::string entity;
  std::string arg;
};

/// The relayed commands POST /api/sat1/ma/<cmd> accepts - exactly the footer's set. Everything the
/// device can do itself (transport, its own volume, shuffle, repeat) stays on /api/sat1/media; these
/// are only what must round-trip through Home Assistant: the favorite button, group membership,
/// another member's volume, and seek, which the Sendspin hub offers no send for.
enum class MaCmd : uint8_t { NONE = 0, LIKE, JOIN, UNJOIN, VOL, SEEK };

/// Bound on the queue. Commands are taps, not drags - volume writes coalesce by entity below - so
/// six outstanding means a browser is misbehaving, and refusal tells it so.
static constexpr size_t WU_MA_QUEUE = 6;

#ifdef USE_MEDIA_PLAYER
/// The transport commands POST /api/sat1/media/<cmd> accepts. The footer's set and no more: stop,
/// mute and the rest exist on the players, but an endpoint nothing renders is surface to maintain
/// and document for no caller. Volume is not here because it travels as a value rather than a verb -
/// see media_pending_vol_. The shuffle and repeat verbs land as media_player commands on the
/// Sendspin group player only, which forwards them over the protocol - the local speaker player has
/// no queue for either to mean anything.
enum class MediaCmd : uint8_t {
  NONE = 0,
  PLAY,
  PAUSE,
  NEXT,
  PREVIOUS,
  SHUFFLE_ON,
  SHUFFLE_OFF,
  REPEAT_OFF,
  REPEAT_ONE,
  REPEAT_ALL,
};
#endif

/// Longest option this will forward, and it is the transport's number rather than a guess.
///
/// The option arrives in the query string, and esp_http_server rejects a URI over
/// CONFIG_HTTPD_MAX_URI_LEN - 512 in this build - before any handler sees it, answering 414. So a cap
/// above what fits would not be a cap at all: the request would fail at the server with a status this
/// code never chose, instead of failing here with an explanation.
///
/// The budget: 21 bytes of path, 6 for the two parameter names, and up to 160 for an entity id, leaves
/// 325 for the option. Percent-encoding can treble a byte, so 100 is the most that is certain to arrive
/// whatever it contains. Pipeline names are typed by people into Home Assistant's Voice assistants page
/// and run to a couple of dozen characters, so this is far from binding in practice.
static constexpr size_t WU_SELECT_OPTION_MAX = 100;

/// Smallest contiguous internal block this will attempt a streamed response in. Below it the request is
/// answered 503 rather than allocated for - see the guard in handleRequest.
///
/// A floor, not a comfort margin, and the difference is the whole design of it. The first version asked
/// for 16KB and refused every streamed route on a wifi build, because that is more contiguous internal
/// memory than this device has - measured at 7,680 bytes largest free block on hardware - so it turned a
/// device that worked into one with no data in the app at all. A guard that fires in normal operation is
/// not a guard.
///
/// So it is sized to what an allocation actually needs. std::string doubles its buffer as it grows, so the
/// largest body here - GET /api/sat1/state, about 2KB with the entity table - peaks at a 4KB buffer while
/// the 2KB one it is copying from is still alive. 6KB clears that and still sits under what the device
/// has, so the guard only speaks when the request really could not have been served. The payload that
/// motivated all of this is not in this set: handle_ha_ needs no heap at all now.
static constexpr size_t WU_STREAM_MIN_BLOCK = 6 * 1024;

/// One row of the key -> "<domain>/<name>" table the frontend reads from GET /api/sat1/state.
/// `key` and `domain` are string literals from generated code; `entity` is resolved to its name
/// lazily, because get_name() is only meaningful once the entity has been constructed.
struct EntityRef {
  const char *key;
  const char *domain;
  EntityBase *entity;
};

#ifdef USE_SAT1_WEB_UI_SOUNDS
/// One device-served sound: the tail of its URL under /api/sat1/sounds/, and the audio_file whose
/// embedded bytes answer it. `name` is a string literal from generated code and carries its own
/// extension ("timer_finished.mp3"), because the device cannot rename what a remote speaker will
/// key its decoder on; the Content-Type is taken from the embedded file_type instead, so the two
/// can never disagree about what the bytes actually are.
struct SoundRef {
  const char *name;
  const audio::AudioFile *file;
};
#endif

/**
 * The web app's HTTP surface: the bundle itself, plus the endpoints web_server cannot cover.
 *
 * Three constraints from the shared server shaped this file, all of them inherited from Phase 1:
 *
 * The httpd task stack is 4352 bytes and there is no YAML knob for it - AsyncWebServer::begin()
 * hardcodes HTTPD_DEFAULT_CONFIG().stack_size + 256. So every JSON response is streamed into an
 * AsyncResponseStream, which accumulates on the heap, rather than built in a stack buffer. The one
 * exception is handle_ha_: that heap is the internal one, and the payload it serves is far too large to
 * copy into it - see the comment there.
 *
 * web_server_idf registers exactly three wildcard URI handlers - GET, POST and OPTIONS - so PATCH
 * and DELETE never reach any handler at all. Writes are POST.
 *
 * ESPHome's response API can only express seven status codes. AsyncWebServerRequest::init_response_
 * maps them through a switch that knows 200, 204, 400, 401, 404, 409 and 422 and defaults everything
 * else to 500 - so beginResponse(304, ...) would send a 500 carrying our cache headers, and a 202
 * would report a failure while still sending its body. handle_index_ therefore reaches past it to
 * httpd_resp_set_status directly, which the operator httpd_req_t*() on the request makes legal;
 * handle_ha_refresh_ takes the other option and answers 200.
 */
class WebUIHandler : public AsyncWebHandler {
 public:
  void set_index(const uint8_t *gz, size_t gz_len) {
    this->index_gz_ = gz;
    this->index_gz_len_ = gz_len;
  }

  /// Must outlive every request: ESP-IDF's httpd_resp_set_hdr stores the pointer rather than
  /// copying, and its own documentation says "make sure that the lifetime of the field value
  /// strings are valid till send function is called". Codegen passes a string literal.
  void set_etag(const char *etag) { this->etag_ = etag; }

  /// The no-sensor card's product photo, WebP straight from codegen - already compressed, so unlike
  /// the bundle there is no gzip layer. Its own asset rather than a data URI in the bundle because at
  /// ~24KB it is over half the bundle's entire gzipped budget, and only devices without a radar ever
  /// fetch it.
  void set_no_sensor_image(const uint8_t *webp, size_t len) {
    this->no_sensor_webp_ = webp;
    this->no_sensor_webp_len_ = len;
  }

  /// The PWA surface: the manifest and the home-screen icons, all PROGMEM like the photo above so
  /// serving them costs no heap. Icons are PNG because iOS accepts nothing else for home-screen
  /// marks, and they live in flash rather than the bundle because PNG does not gzip - inlining one
  /// would spend the entire remaining bundle budget on pixels.
  void set_manifest(const uint8_t *json, size_t len) {
    this->manifest_ = json;
    this->manifest_len_ = len;
  }
  /// `which`: 0 = 192px, 1 = 512px, 2 = the 180px apple-touch-icon.
  void set_icon(uint8_t which, const uint8_t *png, size_t len) {
    if (which < 3) {
      this->icons_[which] = png;
      this->icon_lens_[which] = len;
    }
  }

  /// The session token as the sign-in link carries it, read from the gate at request time so a
  /// regenerate is reflected immediately. GET /api/sat1/state serves it to the Diagnostics Launch
  /// section - behind the gate, so only a signed-in browser ever reads it.
  void set_session_key_fn(std::function<const char *()> fn) { this->session_key_fn_ = std::move(fn); }

  /// Longest gap between two consecutive main-loop iterations since this was last read, in
  /// milliseconds. Written from the main loop, read from the httpd task, and reset by the read -
  /// which is why it is an exchange rather than a load. Diagnostics is the only reader.
  void set_loop_time_source(std::atomic<uint32_t> *max_loop_ms) { this->max_loop_ms_ = max_loop_ms; }

  /// Called only from generated setup code in main.cpp, which runs to completion before the
  /// listener accepts anything, so the table is immutable by the time the httpd task can read it.
  void add_entity(const char *key, const char *domain, EntityBase *entity) {
    this->entities_.push_back({key, domain, entity});
  }

#ifdef USE_SAT1_WEB_UI_SOUNDS
  /// The device-served sounds, from generated code like the entity table above and immutable for
  /// the same reason. Serving them from here rather than S3 is what keeps a mirrored timer ring
  /// working with the internet down - see tts_routing.yaml for the decision and its cost.
  void add_sound(const char *name, const audio::AudioFile *file) { this->sounds_.push_back({name, file}); }
#endif

#ifdef USE_VOICE_ASSISTANT
  void set_voice_assistant(voice_assistant::VoiceAssistant *va) { this->va_ = va; }

  /// Reads the existing `voice_assistant_phase` global rather than mirroring it.
  ///
  /// The plan called for an internal text_sensor fed by the phase triggers, but config/ already
  /// keeps that global and every trigger already maintains it - so a second copy would be one more
  /// thing to keep in step, and would go stale in exactly the situation the panel exists for. A
  /// lambda from YAML is how a C++ component reads a `globals:` entry without depending on the
  /// generated type of the global.
  void set_voice_phase_fn(std::function<int()> fn) { this->voice_phase_fn_ = std::move(fn); }

  /// Called from the on_stt_end and on_tts_start triggers. Runs on the main loop, and the ring is
  /// read from the httpd task, so both sides take the lock.
  void push_utterance(const std::string &text, bool heard);
#endif

  /// Reserves the PSRAM buffer the next payload is written into, or nullptr if it could not grow.
  ///
  /// A buffer handed out rather than a payload handed in, because the payload must never exist as a
  /// std::string. That is not a preference: what Home Assistant renders is several kilobytes, a
  /// std::string puts it on the internal heap, and an allocation that does not fit there aborts the
  /// device rather than failing - which is exactly how this used to reboot on every sync. The caller in
  /// common/web_ui_ha.yaml serializes ArduinoJson's output straight into this buffer instead, so the
  /// bytes go from the API message to PSRAM and nowhere else.
  ///
  /// `capacity` includes room for the terminator, which commit_ha_payload writes.
  ///
  /// A buffer of its own rather than the one being served, so a request in flight cannot see a
  /// half-written payload. Main loop only, like the callback that calls it.
  char *stage_ha_payload(size_t capacity);

  /// Publishes `len` bytes previously written into the staged buffer, as the payload from `rung`, and
  /// returns them NUL-terminated so the caller can read what it just published.
  ///
  /// Split from staging because the two ends belong to different callers: the lambda in YAML knows how
  /// many bytes it wrote, and each rung knows its own number. Runs on the main loop and takes the lock,
  /// since this is where the staged buffer becomes the one requests read.
  ///
  /// The returned pointer stays valid until the next commit, which is also main loop only - so the
  /// component's own read of `aid` out of the payload needs no lock and no second copy. Null if the
  /// commit was refused, which only happens when nothing was staged for it.
  const char *commit_ha_payload(size_t len, int rung);

  /// Starts a paged sync, discarding anything a previous one had staged but not committed.
  ///
  /// common/web_ui_ha.yaml asks for the document a page at a time, because the API's receive buffer
  /// grows to the largest message it has ever handled and is released only once. The pages are byte
  /// slices of one ordered list of fragments rather than structured sub-documents, so appending them
  /// yields the same JSON the single-shot path wrote.
  void begin_ha_pages();

  /// Reserves `len` bytes at the end of the staged payload for one page, or nullptr if PSRAM could
  /// not provide them. Main loop only, like the API callback that calls it.
  char *stage_ha_page(size_t len);

  /// Publishes every page appended since begin_ha_pages as the payload from `rung`, and returns it
  /// NUL-terminated. Null if nothing was staged.
  const char *commit_ha_pages(int rung);

  /// Records that every rung refused, without discarding a payload an earlier sync managed to get.
  /// Stale data with an honest age is more use to the app than nothing.
  void set_ha_failed() { this->ha_rung_ = -1; }

  /// What tts_routing's probe concluded about the "Allow the device to perform Home Assistant
  /// actions" checkbox: 0 unknown, 1 allowed, 2 blocked (the checkbox is off), 3 unverifiable
  /// (Home Assistant predates 2025.12 and answers no action call). Pushed from
  /// tts_routing_status_publish - the one script every verdict transition already runs through -
  /// and served as `actions` on /api/sat1/ha, where it is what lets the app tell "tick this
  /// checkbox" apart from "upgrade Home Assistant" instead of hedging between them. An atomic
  /// because it is written from the main loop and read from the httpd task, the same split as the
  /// refresh flag above.
  void set_ha_actions(int verdict) { this->ha_actions_.store(verdict); }

  /// True once, if a browser has asked for a resync since the last call. An atomic exchange rather
  /// than a scheduler call, because this is set from the httpd task and read from the main loop -
  /// the same split satellite1_radar's engineering-mode gating uses.
  bool take_ha_refresh_request() { return this->ha_refresh_requested_.exchange(false); }

  /// The Music Assistant payload's staging pair, mirroring the Home Assistant one above but
  /// single-shot: the payload is group members and live state, bounded well under one page, so it
  /// arrives as one action reply and needs no page loop. Same PSRAM-only discipline, same
  /// stage-then-swap so a request in flight never reads a half-written payload. Main loop only.
  char *stage_ma_payload(size_t capacity);
  void commit_ma_payload(size_t len);

  /// A browser asked for an MA resync (footer expanded, or after a relayed command).
  bool take_ma_refresh_request() { return this->ma_refresh_requested_.exchange(false); }

  /// Hands back the oldest queued Music Assistant command, if any - the select queue's shape, one
  /// per main-loop iteration so the trigger behind it is never re-entered.
  bool take_ma_write(MaWrite &out);

  /// Hands back the oldest queued select write, if there is one. Runs on the main loop.
  ///
  /// One at a time rather than the whole queue, so the component fires its trigger at most once per
  /// iteration. That removes the question of what an ESPHome automation does when it is re-triggered
  /// while still running - ActionList has no re-entrancy guard, and a YAML author who puts a `delay:`
  /// in this trigger should not be able to turn four writes into a corrupted action list. Four writes
  /// then take four iterations, which on this device is a few tens of milliseconds.
  ///
  /// Checks the flag before taking the lock, so the common case of an idle queue costs one relaxed
  /// load per main-loop iteration and nothing else.
  bool take_select_write(SelectWrite &out);

  /// Set from the component's setup(), before this handler is registered.
  void set_selection(Selection *selection) { this->selection_ = selection; }

#ifdef USE_MEDIA_PLAYER
  /// Both set from generated setup code, before the listener accepts anything.
  ///
  /// Media players are the other thing web_server cannot cover: it registers no media_player
  /// handler at all, so they ride neither /events nor the entity REST API, and the two endpoints
  /// here are the only way a browser sees or moves them. Two players rather than a list because
  /// the preference between them is fixed and named - see active_media_.
  void set_media_player(media_player::MediaPlayer *mp) { this->media_local_ = mp; }
  void set_sendspin_media_player(media_player::MediaPlayer *mp) { this->media_sendspin_ = mp; }

  /// Applies whatever a browser asked for since the last call. Must run on the main loop:
  /// make_call().perform() starts pipeline work on components that assume it, exactly as the wake
  /// word models do.
  void apply_media_requests();

#ifdef USE_SAT1_WEB_UI_SENDSPIN
  /// The Sendspin extras GET /api/sat1/media carries when the hub is configured. All four setters
  /// run on the main loop - the component translates the hub's callbacks, so this file never sees a
  /// sendspin type - and the endpoint reads them from the httpd task, which is why the strings go
  /// through a lock and the numbers through atomics.

  /// Replaces the cached track metadata. Null pointers mean the field is absent, matching the
  /// protocol's own semantics: every metadata message replaces the whole state, so a lost
  /// connection arrives here as four nulls and a zero duration, and the fragment empties.
  void media_set_meta(const char *title, const char *artist, const char *album, const char *art, uint32_t dur_ms);

  /// Replaces the cached controller state: shuffle on/off, repeat 0 off / 1 one / 2 all, the
  /// server's supported-command bitmask (bit n = sendspin command enumerator n), and how far a seek
  /// may target - zero meaning the server offers no seek or the stream's length is unknown.
  void media_set_ctrl(bool shuffle, uint8_t repeat, uint16_t supported, uint32_t seek_max_ms);

  /// The connection dropped and the cached controller state with it.
  void media_clear_ctrl();

  /// The hub's interpolated track position, refreshed by the component once per loop iteration so
  /// the endpoint never calls into the hub from the httpd task.
  void media_set_pos(uint32_t ms) { this->media_pos_ms_.store(ms, std::memory_order_relaxed); }
#endif
#endif

#ifdef USE_MICRO_WAKE_WORD
  /// Set from the component's setup(), before this handler is registered.
  ///
  /// Wake words are the one thing this app controls that is not an entity. micro_wake_word creates no
  /// switch and no select - the models are plain C++ objects, and a model's `internal:` is a model
  /// parameter meaning "do not offer this to Home Assistant", not the usual entity flag. So they never
  /// reach /events or the entity REST API, and until this endpoint the only way to change the active
  /// set was from Home Assistant.
  void set_micro_wake_word(micro_wake_word::MicroWakeWord *mww) { this->mww_ = mww; }

  /// Applies whatever a browser asked for since the last call. Must run on the main loop.
  void apply_wake_word_requests();

  /// Called from the on_wake_word_detected automation, on the main loop. Feeds the ring the test
  /// moment and the Diagnostics history read from.
  void push_wake_detection(const std::string &word);

  /// Whether a "say it now" test window is open. Read from the on_wake_word_detected automation:
  /// while true, a firing is recorded in the ring (which is how the app celebrates it) but the
  /// assistant is not started and no chime plays - the customer asked to hear whether the word
  /// works, not to open a conversation (owner request, September 2026). A ringing timer outranks
  /// the window in YAML, exactly as it outranks the sign-in gate, so "stop" still silences an
  /// alarm mid-test.
  bool wake_test_active() { return millis() < this->wake_test_until_.load(std::memory_order_relaxed); }
#endif

#ifdef USE_SAT1_MWW_LOADER
  /// Set from the component's setup(), before this handler is registered. With a loader present
  /// the wake words endpoint speaks slots - see handle_wake_words_ - and the slot/cutoff POSTs
  /// exist at all.
  void set_wake_loader(mww_runtime_loader::MwwRuntimeLoader *loader) { this->wake_loader_ = loader; }
#endif

  // NOLINTNEXTLINE(readability-identifier-naming)
  bool canHandle(AsyncWebServerRequest *request) const override;
  // NOLINTNEXTLINE(readability-identifier-naming)
  void handleRequest(AsyncWebServerRequest *request) override;

  /// Accumulates a POST body. web_server_idf routes any content type it does not recognise as a form
  /// to whichever handler claims the path, in chunks, and calls handleRequest afterwards - see
  /// AsyncWebServer::handle_raw_body_. That path is the reason the selection is posted as JSON rather
  /// than as form fields: the form branch rejects anything over CONFIG_HTTPD_MAX_REQ_HDR_LEN, which
  /// this build sets to 1024, and a selection can legitimately exceed that.
  // NOLINTNEXTLINE(readability-identifier-naming)
  void handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) override;

 protected:
  /// One route table for canHandle and handleRequest, so the two can never disagree about which
  /// paths exist. Deliberately narrow: this handler is first in web_server_base's vector, so
  /// anything it claims by accident it steals from /events, the entity REST API or /radar_tuner.
  ///
  /// The optional routes are compiled out with the component they serve, rather than kept and left
  /// unreachable, so that -Wswitch still means something: an enumerator that exists in every build but
  /// is only handled in some turns a useful warning into noise that has to be suppressed. The values
  /// are never stored or sent, so it does not matter that they renumber with the config.
  enum class Route : uint8_t {
    NONE = 0,
    INDEX,
    ASSET_NO_SENSOR,
    MANIFEST,
    ICON_192,
    ICON_512,
    ICON_180,
    STATE,
#ifdef USE_VOICE_ASSISTANT
    VOICE,
#endif
    HA,
    HA_REFRESH,
    HA_SELECT,
    MA,
    MA_REFRESH,
    MA_SET,
    SEL,
    SEL_SET,
#ifdef USE_MICRO_WAKE_WORD
    WAKE_WORDS,
#ifndef USE_SAT1_MWW_LOADER
    // The old per-index arm/disarm toggle. Compiled out when the loader owns the slots, because a
    // raw toggle would be undone by the loader's reconciliation on the next iteration.
    WAKE_WORDS_SET,
#endif
#endif
#ifdef USE_SAT1_MWW_LOADER
    WAKE_SLOT_SET,
    WAKE_CUTOFF_SET,
    WAKE_TUNE_SET,
#endif
#ifdef USE_MEDIA_PLAYER
    MEDIA,
    MEDIA_SET,
#endif
#ifdef USE_SAT1_WEB_UI_SOUNDS
    SOUND,
#endif
  };

  static Route match_route_(AsyncWebServerRequest *request);

  /// True for the routes whose body is built in an AsyncResponseStream, and which therefore cannot be
  /// answered at all without internal heap. Read by handleRequest's low-memory guard.
  static bool route_streams_(Route route);

  /// Answers 503 without allocating anything, for a request that arrived with no room to serve it.
  void send_low_memory_(AsyncWebServerRequest *request);

  void handle_index_(AsyncWebServerRequest *request);
  void handle_no_sensor_(AsyncWebServerRequest *request);
  /// One handler for the manifest and all three icons: the same PROGMEM send with a different
  /// pointer and content type each.
  void handle_pwa_asset_(AsyncWebServerRequest *request, const uint8_t *data, size_t len, const char *type);
#ifdef USE_SAT1_WEB_UI_SOUNDS
  /// A device-served sound, PROGMEM like the assets above but with single-range support on top:
  /// Cast refuses media whose origin cannot answer a Range request, and Sonos may probe with one.
  void handle_sound_(AsyncWebServerRequest *request);
#endif
  void handle_state_(AsyncWebServerRequest *request);
#ifdef USE_VOICE_ASSISTANT
  void handle_voice_(AsyncWebServerRequest *request);
#endif
  void handle_ha_(AsyncWebServerRequest *request);
  void handle_ha_refresh_(AsyncWebServerRequest *request);
  void handle_ha_select_(AsyncWebServerRequest *request);

  /// True if the last Home Assistant payload contained `entity` as a complete JSON string.
  ///
  /// The quotes are part of the search, which is what makes it an exact token match rather than a
  /// prefix one - without them "select.x_assistant" would be found inside "select.x_assistant_2".
  /// A find rather than a parse, for the reason the payload buffer's own comment gives.
  bool ha_payload_names_(const std::string &entity);
  void handle_ma_(AsyncWebServerRequest *request);
  void handle_ma_refresh_(AsyncWebServerRequest *request);
  void handle_ma_set_(AsyncWebServerRequest *request);

  /// The MA twin of ha_payload_names_, over the members payload, for entities the big payload does
  /// not carry. Same find-with-quotes, same reasons.
  bool ma_payload_names_(const std::string &entity);
  void handle_sel_(AsyncWebServerRequest *request);
  void handle_sel_set_(AsyncWebServerRequest *request);
#ifdef USE_MICRO_WAKE_WORD
  void handle_wake_words_(AsyncWebServerRequest *request);
#ifndef USE_SAT1_MWW_LOADER
  void handle_wake_words_set_(AsyncWebServerRequest *request);
#endif
#endif
#ifdef USE_SAT1_MWW_LOADER
  void handle_wake_slot_set_(AsyncWebServerRequest *request);
  void handle_wake_cutoff_set_(AsyncWebServerRequest *request);
  void handle_wake_tune_set_(AsyncWebServerRequest *request);
#endif
#ifdef USE_MEDIA_PLAYER
  void handle_media_(AsyncWebServerRequest *request);
  void handle_media_set_(AsyncWebServerRequest *request);

  /// Which player a command or the card is about, resolved fresh each time it is asked.
  ///
  /// The plan's rule is "prefer the Sendspin group while it is actively playing", and the ladder here
  /// is that rule plus the two edges it leaves open. A *paused* Sendspin stream still owns the card -
  /// otherwise pressing pause would flip the card to the idle local player and the play button that
  /// promised to resume would start the wrong thing - but it loses to a local player that is actually
  /// making sound, so a Home Assistant stream started over a paused group session is the one on
  /// screen. Local ANNOUNCING counts as sound for the same reason.
  ///
  /// The paused rung is currently unreachable, and knowing why matters more than removing it: the
  /// Sendspin protocol has no paused state, so a paused group arrives here as IDLE. The frontend
  /// carries that gap - see the `src` parameter on POST - and the rung stays for the day upstream
  /// learns to say PAUSED, at which point it starts working instead of starting to lie.
  media_player::MediaPlayer *active_media_() const;

  /// The player a request named, or the active one when it named none. Null when it named one this
  /// build does not have, which the caller treats as a stale intention to drop.
  media_player::MediaPlayer *resolve_media_(uint8_t src) const;
#endif

  const uint8_t *index_gz_{nullptr};
  size_t index_gz_len_{0};
  const char *etag_{nullptr};
  const uint8_t *no_sensor_webp_{nullptr};
  size_t no_sensor_webp_len_{0};
  const uint8_t *manifest_{nullptr};
  size_t manifest_len_{0};
  const uint8_t *icons_[3]{nullptr, nullptr, nullptr};
  size_t icon_lens_[3]{0, 0, 0};
  std::function<const char *()> session_key_fn_{};
  std::atomic<uint32_t> *max_loop_ms_{nullptr};
  std::vector<EntityRef> entities_;
#ifdef USE_SAT1_WEB_UI_SOUNDS
  std::vector<SoundRef> sounds_;
#endif

  /// The Home Assistant payload, in PSRAM. It is bounded on Home Assistant's side rather than here -
  /// see the size cap in common/web_ui_ha.yaml - but the bound is 24KB, which has no business in a
  /// 165KB internal heap shared with audio buffers.
  ///
  /// Held as bytes rather than parsed: the frontend is the only consumer and it wants JSON, so
  /// parsing here would cost a DOM and then a re-serialization to hand back exactly what arrived.
  /// The buffer only ever grows, since a re-sync of the same installation is nearly the same size and
  /// churning PSRAM to save a few hundred bytes would fragment it for no gain.
  RAMAllocator<char> ha_alloc_{RAMAllocator<char>::ALLOC_EXTERNAL};
  char *ha_buf_{nullptr};
  size_t ha_len_{0};
  size_t ha_cap_{0};

  /// Where the next payload is written, also in PSRAM, and the reason a sync costs no copy: commit
  /// swaps this with the buffer above rather than moving bytes between them. So the pair alternates,
  /// and after the first two syncs neither one allocates again.
  char *ha_stage_{nullptr};
  size_t ha_stage_cap_{0};

  /// Write offset into the staging buffer: how much the pages received so far have filled.
  size_t ha_stage_len_{0};

  /// Grows the staging buffer to `capacity` bytes, preserving its contents, or nullptr if PSRAM
  /// could not provide it. Shared by the single-shot and paged paths, which differ only in whether
  /// they reset ha_stage_len_ first.
  char *ha_stage_grow_(size_t capacity);
  /// Uptime in seconds when the payload arrived, so the app can show its age and decide to resync.
  /// Zero means nothing has ever arrived, which is why the endpoint reports an age of -1 for it.
  uint32_t ha_at_{0};
  int ha_rung_{0};
  /// See set_ha_actions. Separate from ha_rung_ because the two answer different questions: the rung
  /// says what the last sync managed, this says why a refused one was refused.
  std::atomic<int> ha_actions_{0};
  Mutex ha_lock_;
  std::atomic<bool> ha_refresh_requested_{false};

  /// Select writes queued by a browser, waiting for the main loop.
  ///
  /// Deferred for the same reason the refresh is: the endpoint runs on the httpd task, and what this
  /// ultimately starts is a Home Assistant action call, which has to be issued from the main loop.
  ///
  /// Coalesced by entity rather than appended blindly. Dragging a dropdown open and picking twice
  /// should end with the second choice and one action call, not two calls racing to decide what Home
  /// Assistant ends up storing. A queue rather than the wake words' bitmask because the payload here is
  /// two strings, and because the entities are Home Assistant's rather than a fixed local list.
  std::vector<SelectWrite> select_queue_;
  Mutex select_lock_;
  /// Read once per main-loop iteration, so it is worth not taking the lock to find out the queue is
  /// empty. Only ever set true under the lock, and cleared under it when the last entry leaves.
  std::atomic<bool> select_pending_{false};

  /// The Music Assistant payload and its stage, PSRAM like the Home Assistant pair and swapped the
  /// same way. Small (group members and live state), but the discipline is not about size alone:
  /// the internal heap is the one audio competes for, and this arrives while audio plays.
  char *ma_buf_{nullptr};
  size_t ma_len_{0};
  size_t ma_cap_{0};
  char *ma_stage_{nullptr};
  size_t ma_stage_cap_{0};
  uint32_t ma_at_{0};
  Mutex ma_lock_;
  std::atomic<bool> ma_refresh_requested_{false};

  /// Music Assistant commands queued by a browser, waiting for the main loop - the select queue's
  /// twin. Volume and seek writes coalesce by kind and entity, so a drag ends as one action call.
  std::vector<MaWrite> ma_queue_;
  Mutex ma_queue_lock_;
  std::atomic<bool> ma_pending_{false};

  Selection *selection_{nullptr};

  /// Ceiling on a posted selection, comfortably above what the store itself accepts, so an oversized
  /// body is rejected on its own terms rather than by running the heap down first.
  static const size_t SEL_BODY_MAX = 4096;

  /// The POST body being assembled. handleBody can be called several times for one request, and
  /// handleRequest runs afterwards on the same httpd task, so no lock is needed between them - but the
  /// buffer is cleared at index 0 so a connection that died mid-body cannot leave a fragment behind to
  /// be parsed as the front of the next one.
  std::string body_;

#ifdef USE_MICRO_WAKE_WORD
  micro_wake_word::MicroWakeWord *mww_{nullptr};

  /// A wake word toggle is recorded here and applied from the main loop, never from the httpd task.
  ///
  /// Three reasons, any one of them sufficient. micro_wake_word runs inference on its own task and
  /// reads each model's `enabled_`, which is a plain bool - writing it from here would be a data race.
  /// WakeWordModel::enable() also saves to flash, and an NVS write on the server task can stall it.
  /// And ESPHome's own API for this component says model changes must come from the main loop, because
  /// the inference task is paused at a safe point first.
  ///
  /// Coalesced rather than queued: one bit per model in `mask`, and the wanted state in `on`. Two
  /// presses of the same switch before the next loop iteration collapse to the last one, which is the
  /// correct answer, and no request can be dropped for want of queue space. Written mask last, so the
  /// loop never sees a bit set without its state beside it. 32 models is far past anything that fits
  /// in the tensor arenas, and the index is bounds-checked against it anyway.
  std::atomic<uint32_t> ww_pending_mask_{0};
  std::atomic<uint32_t> ww_pending_on_{0};

  /// The detection ring, written from the main loop (on_wake_word_detected) and read from the httpd
  /// task, so both sides take the lock - the transcript ring's exact shape. `seq` counts every
  /// firing ever, so the test moment can tell a fresh detection from the one it already showed
  /// without comparing timestamps across two clocks.
  std::vector<WakeDetection> detections_;
  uint32_t detection_seq_{0};
  Mutex detection_lock_;

  /// millis() deadline of the open test window, 0 when none. Written from the httpd task (the POST
  /// that opens it), read from the main loop - an atomic, like the refresh flags.
  std::atomic<uint32_t> wake_test_until_{0};
#endif

#ifdef USE_SAT1_MWW_LOADER
  mww_runtime_loader::MwwRuntimeLoader *wake_loader_{nullptr};
#endif

#ifdef USE_VOICE_ASSISTANT
  voice_assistant::VoiceAssistant *va_{nullptr};
  std::function<int()> voice_phase_fn_{};
  std::vector<Utterance> transcript_;
  Mutex transcript_lock_;
#endif

#ifdef USE_MEDIA_PLAYER
  media_player::MediaPlayer *media_local_{nullptr};
  media_player::MediaPlayer *media_sendspin_{nullptr};

  /// A media command is recorded here and applied from the main loop, never from the httpd task -
  /// the same split as the wake words, for the same first reason: make_call().perform() starts
  /// pipeline work on the target component, which nothing there expects from another task.
  ///
  /// Coalesced last-writer-wins rather than queued, and split in two because the two writes are
  /// independent: a volume drag mid-pause should not cancel the pause, and the pause should not
  /// discard the volume. Command 0 is none, otherwise MediaCmd; volume -1 is none, otherwise
  /// 0..100. Two taps before the next loop iteration collapse to the last, which for transport
  /// buttons is the correct answer.
  std::atomic<uint8_t> media_pending_cmd_{0};
  std::atomic<int16_t> media_pending_vol_{-1};

  /// Which player each pending write is for: 0 resolve as active, 1 the Sendspin group, 2 local.
  ///
  /// Explicit targets exist because "active" and "what the card shows" can genuinely disagree, and
  /// hardware found the case: the Sendspin protocol has no paused state - SendspinPlaybackState is
  /// PLAYING or STOPPED, nothing else - so a paused group stream reports as an idle player and the
  /// active resolution falls through to local. The card that did the pausing is the only witness
  /// that the group is resumable, so it names its target and this code believes it. Written before
  /// their command/volume partner, which is the flag apply reads.
  std::atomic<uint8_t> media_pending_cmd_src_{0};
  std::atomic<uint8_t> media_pending_vol_src_{0};

#ifdef USE_SAT1_WEB_UI_SENDSPIN
  /// The metadata fragment GET /api/sat1/media appends: pre-escaped `,"title":"..."` pairs, built
  /// once per metadata message on the main loop and copied out under the lock at request time.
  /// Pre-built because titles need JSON escaping and the request path should pay a memcpy, not an
  /// escape walk. PSRAM-backed so a long title never sits on the internal heap for the life of a
  /// track.
  std::basic_string<char, std::char_traits<char>, RAMAllocator<char>> media_meta_json_;
  Mutex media_meta_lock_;

  /// The response body the endpoint assembles: core + numbers + fragment. PSRAM and persistent for
  /// the same reason the HA payload's buffers are - httpd_resp_send copies synchronously, the
  /// httpd task is single, and growing once beats allocating per poll. Touched only from the httpd
  /// task.
  char *media_body_{nullptr};
  size_t media_body_cap_{0};

  /// Controller state and track timing, written on the main loop and read raw from the httpd task -
  /// each is independently atomic, and a poll that catches a half-applied update is corrected one
  /// second later by the next poll.
  std::atomic<uint32_t> media_pos_ms_{0};
  std::atomic<uint32_t> media_dur_ms_{0};
  std::atomic<uint32_t> media_seek_max_{0};
  std::atomic<uint16_t> media_sup_{0};
  std::atomic<uint8_t> media_shuffle_{0};
  std::atomic<uint8_t> media_repeat_{0};
  std::atomic<uint8_t> media_ctrl_ok_{0};
#endif
#endif
};

}  // namespace satellite1_web_ui
}  // namespace esphome
