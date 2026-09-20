#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
// For the ESPHOME_LOG_LEVEL comparison in tune_capable(): the tuner's score channel is the debug
// log, so a build compiled without DEBUG must report itself unable rather than show empty meters.
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include "esphome/components/http_request/http_request.h"
#include "esphome/components/micro_wake_word/micro_wake_word.h"
#include "esphome/components/micro_wake_word/model_data.h"
#include "esphome/components/micro_wake_word/streaming_model.h"

namespace esphome {
namespace mww_runtime_loader {

/// How many wake words this device runs at once. A product decision, not a guess: the firmware has
/// always shipped two words plus the internal `stop`, Home Assistant pairs exactly two with an
/// assistant of their own, and the web app's whole layout is a Primary and a Secondary picker.
static constexpr uint8_t WL_SLOTS = 2;

/// Longest slot spec (a manifest URL, or a built-in model id). Sized to fit the preference struct
/// comfortably; GitHub raw URLs to deeply nested models run ~120 characters, so 224 is headroom,
/// not a squeeze.
static constexpr size_t WL_SPEC_MAX = 224;

/// Caps on what the loader will fetch. A manifest is a few hundred bytes of JSON; the shipped
/// models are 52-64KB of tflite. Both caps are several times the observed sizes, so hitting one
/// means the URL is not what the customer thought it was.
static constexpr size_t WL_MANIFEST_MAX = 8 * 1024;
static constexpr size_t WL_MODEL_MAX = 512 * 1024;
static constexpr size_t WL_ARENA_MAX = 128 * 1024;

/// What a slot is doing right now. READY covers both "listening" and "empty" - an empty slot is
/// distinguished by an empty spec, not by a state of its own.
enum SlotState : uint8_t {
  SLOT_READY = 0,
  SLOT_DOWNLOADING = 1,
  SLOT_ERROR = 2,
  SLOT_WAITING = 3,  // holds a URL, waiting for the network (boot re-download)
};

/// Why a swap failed, one code per rung of the validation ladder. The frontend owns the words;
/// these numbers are the contract. Renumbering them breaks every shipped web app, so append only.
enum SlotError : uint8_t {
  ERR_NONE = 0,
  ERR_FETCH = 1,        // manifest URL unreachable or non-200
  ERR_NOT_MANIFEST = 2, // body is not a microWakeWord manifest (bad JSON, wrong type, no micro block)
  ERR_VERSION = 3,      // manifest version != 2
  ERR_STEP = 4,         // feature_step_size does not match the compiled frontend
  ERR_TOO_NEW = 5,      // minimum_esphome_version is newer than this firmware
  ERR_TOO_BIG = 6,      // model or tensor arena over the caps above
  ERR_DOWNLOAD = 7,     // model fetch failed or arrived truncated
  ERR_INVALID = 8,      // bytes are not a usable TFLite model
  ERR_REFUSED = 9,      // micro_wake_word refused the model (duplicate id, task not pausable)
  ERR_NO_MEMORY = 10,   // could not allocate the model buffer
};

/// What the wake words endpoint serves per slot, copied out under the view lock so the httpd task
/// never reads strings the main loop is mutating.
/// A slot's persisted sensitivity: 0 means the model's own tuning, and anything from WL_TUNED_MIN
/// up is a measured threshold the Wake Word Tuner applied (quantized like every cutoff, 255 = 1.0).
/// The gap below WL_TUNED_MIN keeps a tuned value distinguishable from the retired step numbers a
/// dev build may have persisted, and no useful threshold lives that low anyway.
static constexpr uint8_t WL_TUNED_MIN = 100;

/// The probe floor a tune session drops the model to (~0.42): low enough that nearly every attempt
/// - and every room false-trigger - produces a scored detection event, high enough that the ring
/// is not spammed by noise a real threshold could never sit below.
static constexpr uint8_t WL_TUNE_FLOOR = 107;

/// How long a session lives past its last keepalive. The app re-posts every 20 seconds while the
/// panel is open, so this only expires when the browser is gone - and expiry restores the cutoff.
static constexpr uint32_t WL_TUNE_TTL_MS = 120 * 1000;

struct SlotView {
  std::string spec;   // "" empty, built-in id, or manifest URL
  std::string word;   // resolved phrase, "" until known
  std::string id;     // loaded model id, "" until loaded
  uint8_t state{SLOT_READY};
  uint8_t error{ERR_NONE};
  uint8_t cutoff{0};  // persisted sensitivity: 0 = model default, >= WL_TUNED_MIN = tuned threshold
  bool runtime{false};
};

/// One scored event during a tune session: a detection's peak and sliding-average probability, or
/// a VAD rejection (the wake model fired but the voice-activity model did not take it for speech).
struct TuneEvent {
  uint8_t peak;
  uint8_t avg;
  bool vad_blocked;
  uint32_t at_ms;
};

/// What the wake words endpoint serves while a session is live.
struct TuneView {
  int8_t slot{-1};  // -1 when no session is active
  uint32_t seq{0};  // total events ever recorded, so the app can spot new ones across polls
  std::vector<TuneEvent> events;
};

/// How many scored events a session keeps. A tune is a quiet phase plus a handful of utterances;
/// two dozen covers a noisy room's spurious hits without growing into a recording.
static constexpr size_t WL_TUNE_RING = 24;

/// One compiled-in model, mirrored at setup so the endpoint can list the picker's "Included" group
/// without touching micro_wake_word from the httpd task.
struct BuiltinInfo {
  std::string id;
  std::string word;
};

/**
 * Downloads microWakeWord models at runtime and keeps the device's two wake word slots equal to
 * what the customer chose in the web app.
 *
 * The slot model is the whole design. The device runs at most two wake words, so there is no
 * collection to manage: writing a slot with a built-in id enables it, writing a manifest URL
 * downloads the model into PSRAM and hands it to micro_wake_word's runtime API, and whatever the
 * slot previously held is disabled - and freed, if it was a downloaded model. A failed download
 * leaves the previous word active, so a bad URL can never leave the device deaf.
 *
 * Models are deliberately not stored in flash (owner decision, September 2026): the slot remembers
 * the manifest URL and the model is fetched again on boot, with backoff, in a "waiting for
 * network" state the app can show honestly.
 *
 * Threading: the two queue_* methods are the only ones safe off the main loop (the web handler
 * calls them from the httpd task). Downloads run on a short-lived FreeRTOS task that touches only
 * its Job and the HTTP client; everything that mutates micro_wake_word happens in loop(), which is
 * what its runtime API requires.
 */
class MwwRuntimeLoader : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // After micro_wake_word (AFTER_CONNECTION = -100? no: LATE). Default priority is fine: models are
  // constructed in generated code before any setup() runs, so setup order does not matter here.

  void set_micro_wake_word(micro_wake_word::MicroWakeWord *mww) { this->mww_ = mww; }
  void set_http_request(http_request::HttpRequestComponent *http) { this->http_ = http; }

  /* ---- httpd-task-safe API, called by satellite1_web_ui ---- */

  /// Asks slot `i` to hold `spec`: "" or "none" empties it, a built-in model id enables that
  /// model, an http(s) URL downloads that manifest. Returns false when the request is malformed
  /// or names what the other slot already holds; queued otherwise and applied from loop().
  bool queue_slot(uint8_t i, const std::string &spec);

  /// Asks for slot `i`'s sensitivity: 0 for the model's own tuning, or a measured threshold from
  /// the tuner (WL_TUNED_MIN..254). Anything else is refused.
  bool queue_cutoff(uint8_t i, uint8_t value);

  /// Opens or keeps alive (`on`) / closes (`!on`) a tune session on slot `i`. While a session is
  /// live the slot's model runs at WL_TUNE_FLOOR and every detection's scores land in the tune
  /// ring; closing - or the keepalive expiring - restores the configured cutoff. One session at a
  /// time; opening on another slot moves it.
  bool queue_tune(uint8_t i, bool on);

  /// Whether this build can score attempts at all: the log-listener hook is compiled in and the
  /// detection lines exist (DEBUG compiled into the logger). False means the tuner UI should say
  /// "this build cannot score attempts" rather than show an empty meter.
  static bool tune_capable() {
#if defined(USE_LOG_LISTENERS) && ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    return true;
#else
    return false;
#endif
  }

  /// Copies the live tune session state for the endpoint. Safe from the httpd task.
  void tune_snapshot(TuneView &out);

  /// Copies the current slot views. `dl`/`total` report the in-flight download, slot in `dl_slot`.
  void snapshot(SlotView out[WL_SLOTS], uint32_t &dl, uint32_t &total, int &dl_slot);

  /// The compiled-in models, fixed after setup, so the endpoint can serve the "Included" group.
  const std::vector<BuiltinInfo> &builtins() const { return this->builtins_; }

 protected:
  /// One download, inputs written by the main loop before the task starts, outputs written by the
  /// task and read by the main loop only after `done` - so no field needs a lock, only the flags
  /// are atomic.
  struct Job {
    uint8_t slot{0};
    bool boot{false};
    std::string manifest_url;

    uint8_t error{ERR_NONE};
    std::string word;
    std::vector<std::string> langs;
    uint8_t cutoff{247};  // quantized; manifest's probability_cutoff * 255
    uint32_t window{5};
    uint32_t arena{30000};
    std::shared_ptr<micro_wake_word::ModelData> data;

    std::atomic<uint32_t> bytes{0};
    std::atomic<uint32_t> total{0};
    std::atomic<bool> done{false};
    std::atomic<bool> active{false};
  };

  struct Slot {
    std::string spec;
    /// A URL being fetched for this slot. Non-empty only mid-swap: the old spec/model stay live
    /// until the download succeeds, which is what lets a failed swap keep the previous word.
    std::string pending;
    std::string word;      // the loaded word's phrase, for the view and the logs
    uint8_t cutoff{0};     // 0 = the model's own tuning, >= WL_TUNED_MIN = tuned threshold
    uint8_t state{SLOT_READY};
    uint8_t error{ERR_NONE};
    std::string model_id;  // id of the model this slot has loaded/enabled, "" when none
    // Boot re-download bookkeeping. millis_64() so a deadline can never sit on the wrong side of
    // the 49.7-day millis() wrap - these devices stay up for months.
    uint64_t next_retry_ms{0};
    uint8_t retries{0};
  };

  /// What the preference slot stores: the spec and the sensitivity. Plain bytes, because
  /// ESPPreferenceObject wants a trivially copyable struct. The byte briefly held step numbers
  /// (1..4) on dev builds; anything below WL_TUNED_MIN that is not 0 reads as the model default.
  struct SlotPrefData {
    char spec[WL_SPEC_MAX];
    uint8_t cutoff;
    uint8_t used;  // 1 once ever written, so a fresh device is told apart from an emptied slot
  } __attribute__((packed));

  static void job_task(void *param);
  void run_job_();
  /// One HTTP GET into `out`, capped at `cap`. Returns the SlotError verdict.
  uint8_t fetch_(const std::string &url, std::string &out, size_t cap);

  void start_job_(uint8_t slot, bool boot);
  void finalize_job_();
  void apply_request_(uint8_t i, const std::string &spec);
  void apply_cutoff_(uint8_t i, uint8_t value);
  /// Drops the native API connections a moment from now, so Home Assistant reconnects and
  /// re-reads the wake word list. Called when the advertised set changes - a downloaded word
  /// arriving or leaving - never for changes Home Assistant itself originated.
  void nudge_ha_();

  /* ---- the tune session, main loop unless noted ---- */
  void apply_tune_(uint8_t i, bool on);
  void end_tune_(bool expired);
  /// Puts a slot's model back on its configured threshold: the tuned value, or the model default.
  void apply_configured_cutoff_(Slot &s);
  /// The logger hook (any task). Cheap tag test first; parses the two pinned detection lines and
  /// feeds the tune ring only while a session is live.
  static void log_callback_(void *self, uint8_t level, const char *tag, const char *message, size_t len);
  void on_mww_log_(const char *message);
  /// Disables (and for runtime models, removes and frees) whatever `slot` currently has loaded.
  void unload_slot_(Slot &s);
  /// Enforces "enabled iff in a slot" over the compiled models and adopts changes Home Assistant
  /// made through on_set_configuration, which flips enable flags underneath us.
  void reconcile_ha_();
  void save_slot_(uint8_t i);
  void publish_view_();
  /// The compiled model with this id, or nullptr. Main loop only.
  micro_wake_word::WakeWordModel *builtin_by_id_(const std::string &id);
  bool is_builtin_(const std::string &spec);
  static std::string runtime_id_(uint8_t slot);

  micro_wake_word::MicroWakeWord *mww_{nullptr};
  http_request::HttpRequestComponent *http_{nullptr};

  Slot slots_[WL_SLOTS];
  ESPPreferenceObject prefs_[WL_SLOTS];
  std::vector<BuiltinInfo> builtins_;

  // Requests from the httpd task, coalesced per slot: the newest write wins, which is the right
  // answer for a picker someone is changing their mind in.
  Mutex req_lock_;
  bool slot_req_[WL_SLOTS]{false, false};
  std::string slot_req_spec_[WL_SLOTS];
  int16_t cutoff_req_[WL_SLOTS]{-1, -1};
  // A queued tune open/keepalive/close: -1 none, otherwise slot * 2 + (on ? 1 : 0).
  std::atomic<int8_t> tune_req_{-1};
  std::atomic<bool> req_pending_{false};

  /* ---- the live tune session ---- */
  // The slot being tuned, -1 when none. Atomic because the log callback reads it from whichever
  // task produced the line (the two lines it parses happen to come from the main loop, but the
  // guard must be safe for every line that merely shares the tag).
  std::atomic<int8_t> tune_slot_{-1};
  // millis_64(): wrap-proof, like every deadline here.
  uint64_t tune_deadline_ms_{0};
  // The tuned word and the ring, shared between the log callback's writer and the endpoint's
  // reader. tune_word_ is only written at session open, under the same lock the callback compares
  // it under.
  Mutex tune_lock_;
  std::string tune_word_;
  std::vector<TuneEvent> tune_events_;
  uint32_t tune_seq_{0};

  // The published view the endpoint copies from.
  Mutex view_lock_;
  SlotView view_[WL_SLOTS];

  Job job_;
  // Set when reconcile_ha_ should hold off for a beat: right after boot (models are still
  // restoring) and right after our own writes (enable() lands on the next inference pass).
  // millis_64(): wrap-proof, like every deadline here.
  uint64_t reconcile_after_ms_{0};
  // First-loop re-assertion of built-in cutoff overrides - see loop() for why not setup().
  bool overrides_applied_{false};
};

}  // namespace mww_runtime_loader
}  // namespace esphome

#endif  // USE_ESP32
