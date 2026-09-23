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

/// The stop word's pseudo-slot: one past the real slots, accepted by queue_tune and queue_cutoff
/// only. It resolves to the internal `stop` model directly rather than through a slot - the stop
/// word is not swappable, not in the picker, and never advertised to Home Assistant - but its
/// sensitivity is as tunable as any word's (owner request, September 2026), and the tuner's whole
/// session machinery works unchanged once the target model is found.
static constexpr uint8_t WL_STOP = WL_SLOTS;

/// The internal stop model's id, pinned by voice_assistant.yaml (`id: stop`, `internal: true`).
/// Found by id because micro_wake_word's get_wake_words() deliberately excludes internal models;
/// get_model_by_id() does not. A build without the model simply answers "no stop word" everywhere.
static constexpr const char *WL_STOP_ID = "stop";

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
  ERR_FETCH = 1,         // manifest URL unreachable or non-200
  ERR_NOT_MANIFEST = 2,  // body is not a microWakeWord manifest (bad JSON, wrong type, no micro block)
  ERR_VERSION = 3,       // manifest version != 2
  ERR_STEP = 4,          // feature_step_size does not match the compiled frontend
  ERR_TOO_NEW = 5,       // minimum_esphome_version is newer than this firmware
  ERR_TOO_BIG = 6,       // model or tensor arena over the caps above
  ERR_DOWNLOAD = 7,      // model fetch failed or arrived truncated
  ERR_INVALID = 8,       // bytes are not a usable TFLite model
  ERR_REFUSED = 9,       // micro_wake_word refused the model (duplicate id, task not pausable)
  ERR_NO_MEMORY = 10,    // could not allocate the model buffer
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
  std::string spec;  // "" empty, built-in id, or manifest URL
  std::string word;  // resolved phrase, "" until known
  std::string id;    // loaded model id, "" until loaded
  uint8_t state{SLOT_READY};
  uint8_t error{ERR_NONE};
  uint8_t cutoff{0};    // persisted sensitivity: 0 = model default, >= WL_TUNED_MIN = tuned threshold
  uint8_t tn_noise{0};  // last tune's room-noise ceiling (0 = never measured), quantized 0-255
  uint8_t tn_floor{0};  // last tune's quietest attempt (0 = never measured), quantized 0-255
  uint8_t tn_hi{0};     // last tune's loudest attempt, same lifecycle
  bool runtime{false};
};

/// The stop word's endpoint-facing state: its tuned sensitivity and last-tune stats. Enabled/off is
/// deliberately absent - that is the existing `stop_word` switch entity's job, which the app already
/// reads over /events.
struct StopView {
  bool present{false};  // the internal model exists in this build
  uint8_t cutoff{0};
  uint8_t tn_noise{0};
  uint8_t tn_floor{0};
  uint8_t tn_hi{0};
};

/// A transient download body, placed in PSRAM: a manifest can legally run to WL_MANIFEST_MAX (8KB),
/// which has no business on the internal heap even for the seconds a fetch holds it. RAMAllocator
/// falls back to internal on a PSRAM-less board, so this is a placement preference, not a
/// requirement.
using PsramString = std::basic_string<char, std::char_traits<char>, RAMAllocator<char>>;

/// One scored event during a tune session: a detection's peak and sliding-average probability, or
/// a VAD rejection (the wake model fired but the voice-activity model did not take it for speech).
/// `mm` is the utterance's max windowed mean, raised by register samples for ~1.5s after the event
/// lands - the exact quantity determine_detected() compares against the cutoff, and therefore the
/// number the tuner's recommendation math should reason about. Starts at the event's own avg.
struct TuneEvent {
  uint8_t peak;
  uint8_t avg;
  uint8_t mm;
  bool vad_blocked;
  uint32_t at_ms;
};

/// What the wake words endpoint serves while a session is live. `room` is the register's own
/// reading of the room: the max windowed mean of everything that did NOT become an event -
/// sub-floor sounds included, which the event ring structurally cannot see (hardware finding,
/// September 22 2026: blaring music read as "quiet" because it never crossed the probe floor,
/// while the register had been measuring it all along).
struct TuneView {
  int8_t slot{-1};  // -1 when no session is active
  uint32_t seq{0};  // total events ever recorded, so the app can spot new ones across polls
  uint8_t room{0};
  std::vector<TuneEvent> events;
};

/// How many scored events a session keeps. A tune is a quiet phase plus a handful of utterances;
/// two dozen covers a noisy room's spurious hits without growing into a recording.
static constexpr size_t WL_TUNE_RING = 24;

/// One close call: something scored near a word's cutoff and was ignored (owner request, September
/// 2026 - the visible evidence behind both "it almost false-woke" and "it ignored me"). Two
/// sources share the ring: register samples inside the close-call band below the cutoff (scored),
/// and VAD-refused detections from the log listener (vad=1, score 0 - their log line carries no
/// number, and they exist on debug builds only).
struct NearRec {
  std::string word;
  uint8_t score;  // quantized max window mean, 0 for VAD-refused entries
  bool vad;
  uint32_t at_ms;
};

/// How far below a track's active cutoff a sample still counts as a close call: ~5% of the scale.
/// Wider would fill the ring with the room's ordinary chatter; narrower would miss the near-fires
/// the record exists to show.
static constexpr uint8_t WL_NEAR_BAND = 12;

/// Eight, like the detection ring and for the same reason: recent evidence, not a history. RAM
/// only, cleared on restart.
static constexpr size_t WL_NEAR_RING = 8;

/// One remembered graph event: a real firing (kind 0) or a scored close call (kind 1), with the
/// confidence the register measured. Track-scoped rather than word-scoped so it survives reboots
/// and word swaps without carrying strings through NVS. `id` is a monotonically issued tag whose
/// only job is stability: the UI keys each dot's vertical jitter on it, so a new dot landing can
/// never move the old ones (owner's report, September 22 2026 - index-keyed jitter reshuffled the
/// whole graph on every firing). `at_ms` is signed against millis_64, so an entry restored from
/// flash can honestly sit "before boot".
struct TeleRec {
  uint8_t track;
  uint8_t kind;  // 0 = firing, 1 = close call
  uint8_t score;
  uint16_t id;
  int64_t at_ms;
};

/// What tele_snapshot serves per entry: the age already computed, ready for the payload.
struct TeleOut {
  uint32_t ms_ago;
  uint8_t score;
  uint8_t kind;
  uint16_t id;
};

/// The merged ring's cap, all tracks together. Two words plus Stop share it; 32 covers a lively
/// day of firings and close calls inside the 24h window the UI draws.
static constexpr size_t WL_TELE_RING = 32;

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
  /// One last telemetry save, so a clean reboot (OTA, restart button) loses at most the minutes
  /// since the previous save rather than the day.
  void on_shutdown() override { this->save_tele_(); }
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
  /// the tuner (WL_TUNED_MIN..254). `i` may be WL_STOP for the stop word's pseudo-slot. `noise`
  /// and `floor` are the tune session's measured stats (room ceiling, quietest attempt), persisted
  /// alongside the cutoff so the margin bar survives reloads; both 0 when unknown, and cleared
  /// whenever the cutoff resets to 0. Anything else is refused.
  bool queue_cutoff(uint8_t i, uint8_t value, uint8_t noise = 0, uint8_t floor = 0, uint8_t hi = 0);

  /// Opens or keeps alive (`on`) / closes (`!on`) a tune session on slot `i` (WL_STOP for the stop
  /// word). While a session is live the target model runs at WL_TUNE_FLOOR and every detection's
  /// scores land in the tune ring; closing - or the keepalive expiring - restores the configured
  /// cutoff. One session at a time; opening on another slot moves it.
  bool queue_tune(uint8_t i, bool on);

  /// Asks for track `i`'s remembered 24h telemetry - the graph's dots and its hourly smatter - to
  /// be erased (the tuner's Clear history button). Applied from loop(), where the persisted blob
  /// is rewritten; the erase itself is idempotent, so coalescing to the newest request is fine.
  bool queue_clear_history(uint8_t i) {
    if (i > WL_STOP)
      return false;
    this->tele_clear_req_.store(static_cast<int8_t>(i), std::memory_order_relaxed);
    return true;
  }

  /// A wake word fired for real (any word, the stop word included). Called from the YAML
  /// on_wake_word_detected lambda on the main loop - the same place the web UI's detection ring is
  /// fed - so the telemetry can exclude the firing's own scores from "room pressure" without
  /// depending on log lines or build level.
  void notify_detection(const std::string &word);

  /// The score the last real firing drained out of the register (its max window mean - the exact
  /// quantity determine_detected() compared against the cutoff), then 0 until the next firing.
  /// An exchange, so one firing is read once. Called by the web UI's push_wake_detection later in
  /// the same on_wake_word_detected automation, which is what puts a confidence on every dot the
  /// Living Graph draws (v2 owner decision, September 2026) - on every build, not just debug: the
  /// register is the one channel that carries scores without the log listener.
  ///
  /// This drain is also what commits the firing to the persisted 24h ring - deliberately here and
  /// not in notify_detection, because push_wake_detection sits INSIDE the sign-in gate: a firing
  /// consumed as an offline sign-in symbol is an answer to a secret, and it must not land on the
  /// Living Graph any more than in the detection history (the dots carry the same track, time and
  /// confidence the gate exists to hide). Main loop only, like its caller.
  uint8_t take_detection_score();

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

  /// Each track's live room pressure: the max high-water sample over the trailing 24 hours, for
  /// slots 0..WL_SLOTS-1 and the stop word at [WL_STOP]. Safe from the httpd task (plain byte
  /// reads of values the main loop maintains; a torn read is impossible on a byte).
  void hw_snapshot(uint8_t out[WL_SLOTS + 1]);

  /// The same buckets raw, newest hour first (index 0 = the hour in progress, 23 = a day ago), for
  /// the tuner's "wake history" smatter - the v2 tuner draws the room as its actual hourly data
  /// rather than an aggregate (owner decision, September 2026). Safe from the httpd task for the
  /// reason hw_snapshot is.
  void hw_day_snapshot(uint8_t out[WL_SLOTS + 1][24]);

  /// One track's remembered graph events (firings and scored close calls), oldest first, ages
  /// precomputed; entries past the 24h window are omitted. Safe from the httpd task (tele_lock_).
  void tele_snapshot(uint8_t track, std::vector<TeleOut> &out);

  /// The stop word's endpoint-facing state. Safe from the httpd task (view lock).
  void stop_snapshot(StopView &out);

  /// Copies the close-call ring, newest last. Safe from the httpd task (its own lock).
  void near_snapshot(std::vector<NearRec> &out);

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
    std::string word;     // the loaded word's phrase, for the view and the logs
    uint8_t cutoff{0};    // 0 = the model's own tuning, >= WL_TUNED_MIN = tuned threshold
    uint8_t tn_noise{0};  // last tune's room ceiling, 0 = never measured; lives and dies with cutoff
    uint8_t tn_floor{0};  // last tune's quietest attempt, same lifecycle
    uint8_t tn_hi{0};     // last tune's loudest attempt, same lifecycle
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

  /// The 24h telemetry the Living Graphs draw, persisted so a reboot does not blank every graph
  /// (owner request, September 22 2026). One blob: the hourly high-water buckets and the merged
  /// firing/close-call ring, entries as ages in minutes - the device has no wall clock, so on
  /// restore they simply resume that old, plus whatever the downtime added unseen. Saved on every
  /// real firing, on each hourly wheel advance, and at shutdown: a handful of NVS writes a day.
  struct TeleData {
    uint8_t buckets[WL_SLOTS + 1][24];
    uint8_t hour_at;
    uint8_t n;
    struct {
      uint16_t min_ago;
      uint8_t score;
      uint8_t tk;  // track in the low nibble, kind in the high
      uint16_t id;
    } __attribute__((packed)) e[WL_TELE_RING];
    uint16_t next_id;
    uint8_t used;
  } __attribute__((packed));

  /// The tune session's measured stats, persisted so the margin bar survives reloads. A separate
  /// preference per slot rather than new SlotPrefData fields, deliberately: ESPPreferences keys on
  /// the struct size, so growing SlotPrefData would orphan every shipped device's slot data on
  /// upgrade. This struct also serves the stop word (its own hash), whose cutoff rides along
  /// because the stop word has no SlotPrefData at all.
  struct TuneStatsData {
    uint8_t cutoff;  // meaningful for the stop pref only; slots keep theirs in SlotPrefData
    uint8_t noise;
    uint8_t floor;  // the quietest attempt - the "you" band's low edge
    uint8_t hi;     // the loudest attempt - its high edge (hardware finding, September 22 2026:
                    // without it the bar's "you" band had to borrow the noise ceiling and lied)
    uint8_t used;
  } __attribute__((packed));

  /// The endpoint-facing view's storage: SlotView's shape with fixed char arrays instead of
  /// std::string, so the persistent copy of two slots' specs lives inside this object - which
  /// extram_bss places in PSRAM - rather than as internal-heap string bodies. snapshot()
  /// materializes the std::string SlotView the endpoint reads from these, transiently. The word
  /// and id widths are display truncations, not limits: the endpoint only tests `id` for
  /// emptiness, and a phrase past 95 bytes is already not something a picker row can show.
  struct SlotViewStore {
    char spec[WL_SPEC_MAX]{};
    char word[96]{};
    char id[32]{};
    uint8_t state{SLOT_READY};
    uint8_t error{ERR_NONE};
    uint8_t cutoff{0};
    uint8_t tn_noise{0};
    uint8_t tn_floor{0};
    uint8_t tn_hi{0};
    bool runtime{false};
  };

  static void job_task(void *param);
  void run_job_();
  /// One HTTP GET into `out`, capped at `cap`. Returns the SlotError verdict.
  uint8_t fetch_(const std::string &url, PsramString &out, size_t cap);

  void start_job_(uint8_t slot, bool boot);
  void finalize_job_();
  /// Hands the internal heap back the job's transient strings (URL, word, languages). Called at
  /// every finalize exit: the next job rewrites them anyway, but between downloads - which is
  /// almost always - there is no reason to keep a URL's bytes allocated.
  void release_job_strings_();
  void apply_request_(uint8_t i, const std::string &spec);
  void apply_cutoff_(uint8_t i, uint8_t value, uint8_t noise, uint8_t floor, uint8_t hi);
  /// Drops the native API connections a moment from now, so Home Assistant reconnects and
  /// re-reads the wake word list. Called when the advertised set changes - a downloaded word
  /// arriving or leaving - never for changes Home Assistant itself originated.
  void nudge_ha_();

  /* ---- the tune session, main loop unless noted ---- */
  void apply_tune_(uint8_t i, bool on);
  void end_tune_(bool expired);
  /// Puts a slot's model back on its configured threshold: the tuned value, or the model default.
  void apply_configured_cutoff_(Slot &s);
  /// The internal stop model, or nullptr on a build without one. Main loop only.
  micro_wake_word::WakeWordModel *stop_model_();
  /// Puts the stop model back on its configured threshold.
  void apply_stop_cutoff_();
  void save_stop_();
  void save_tune_stats_(uint8_t i);
  /// The register drain: samples every track's high water, feeds the hourly buckets, the close-call
  /// ring and a live tune event's `mm`, honoring the exclusion windows. Main loop, ~every 250ms.
  void sample_high_water_();
  /// Records a close call, coalescing with the newest entry for the same word within 2s.
  void push_near_(const std::string &word, uint8_t score, bool vad);
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
  /// Tune stats per slot, plus the stop word's own state at [WL_STOP] (its cutoff rides the same
  /// struct - see TuneStatsData).
  ESPPreferenceObject tn_prefs_[WL_SLOTS + 1];
  /// The stop word's runtime state, main loop only; published into stop_view_ under view_lock_.
  uint8_t stop_cutoff_{0};
  uint8_t stop_tn_noise_{0};
  uint8_t stop_tn_floor_{0};
  uint8_t stop_tn_hi_{0};
  /// Whether the stop model was enabled before a stop tune session force-enabled it. The scripts
  /// only enable the stop word while something plays, so a session opened in silence would find a
  /// disabled (unloaded) model and measure nothing (hardware finding, September 22 2026); the
  /// session enables it for its own lifetime and puts the prior state back at close.
  bool stop_tune_enabled_it_{false};
  std::vector<BuiltinInfo> builtins_;

  /* ---- register telemetry, main loop only unless noted ---- */
  /// Hourly high-water buckets per track (slots 0..1, stop at WL_STOP): [track][hour-of-day-ish].
  /// The published value is the max over all 24, i.e. "the loudest the room got in the last day".
  /// Plain bytes read by hw_snapshot from the httpd task - byte reads cannot tear.
  uint8_t hw_buckets_[WL_SLOTS + 1][24]{};
  uint8_t hour_at_{0};        // which bucket is "now", advanced (and the new bucket cleared) hourly
  uint64_t hour_started_{0};  // millis_64() when the current bucket opened
  uint64_t next_sample_ms_{0};
  /// Exclusion deadlines per track: samples drained-and-discarded until then. Set +/-2s around a
  /// real detection (notify_detection) so a legitimate wake never reads as room pressure.
  uint64_t hw_suppress_until_[WL_SLOTS + 1]{};
  /// Samples on probation before they may become room pressure, one per track - the standing
  /// twin of tune_pend_ (same bug, same fix; owner's report, September 23 2026: a word said once
  /// before its first tune painted an 87% hourly bucket, which then failed the placement). A
  /// firing inside the window proves the sample was the word's own onset and voids it; ~1.6
  /// eventless seconds commit it to the bucket, the close-call band and the persisted ring.
  uint8_t hw_pend_[WL_SLOTS + 1]{};
  uint64_t hw_pend_at_[WL_SLOTS + 1]{};
  /// Until when reconcile_ha_ RE-ASSERTS the slots over Home Assistant instead of adopting its
  /// flips: set whenever our own advertised set changes. The nudge makes Home Assistant reconnect,
  /// and on reconnect it re-asserts its PRE-change selects - racing the app's pairing write - and
  /// adopting that stale set emptied a slot and freed the very model the write was about to name
  /// (owner's report, September 23 2026: the downloaded word's neighbour vanished).
  uint64_t assert_until_ms_{0};
  /// The last real firing's drained register value and its track - see take_detection_score().
  /// Both are cleared at the top of every notify_detection, so a firing whose drain never came
  /// (the sign-in gate consumed it, or the score never stored) cannot be misattributed to the
  /// next firing's dot. Atomic out of caution only: both writer (notify_detection) and reader
  /// (push_wake_detection via the web handler) run on the main loop today, but nothing in this
  /// class should trust that forever.
  std::atomic<uint8_t> last_det_score_{0};
  std::atomic<int8_t> last_det_track_{-1};

  Mutex near_lock_;
  std::vector<NearRec> near_;

  /* ---- the persisted 24h telemetry ring (see TeleData) ---- */
  /// Pushes one event, drops what has aged out of the window, and issues its stable id. `save`
  /// writes the blob too - firings save (they are rare and the dot matters); close calls ride the
  /// hourly save instead (they can be frequent and the hour's loss is survivable). Main loop only.
  void push_tele_(uint8_t track, uint8_t kind, uint8_t score, bool save);
  void save_tele_();
  void load_tele_();
  /// Erases track `t`'s remembered events, hourly smatter and pending sample, persisting the
  /// erasure. Called by the Clear history button - and by every word change on a slot, because the
  /// telemetry is track-scoped and a newcomer must not inherit its predecessor's history (owner's
  /// report, September 23 2026: a freshly downloaded word opened its first tune wearing an 87%
  /// amber dot it had never earned).
  void clear_track_tele_(uint8_t t);
  Mutex tele_lock_;
  std::vector<TeleRec> tele_;
  uint16_t tele_id_{1};
  ESPPreferenceObject tele_pref_;
  /// A queued Clear history: the track to erase, -1 none. Written from the httpd task.
  std::atomic<int8_t> tele_clear_req_{-1};

  // Requests from the httpd task, coalesced per slot: the newest write wins, which is the right
  // answer for a picker someone is changing their mind in. The spec rides a fixed array (storage
  // inside this PSRAM-resident object) rather than a std::string whose URL-sized body would land
  // on the internal heap; queue_slot bounds the length before writing.
  Mutex req_lock_;
  bool slot_req_[WL_SLOTS]{false, false};
  char slot_req_spec_[WL_SLOTS][WL_SPEC_MAX]{};
  // One extra element for the stop pseudo-slot, whose cutoff/tune ride the same coalescing queue.
  int16_t cutoff_req_[WL_SLOTS + 1]{-1, -1, -1};
  uint8_t cutoff_req_n_[WL_SLOTS + 1]{};
  uint8_t cutoff_req_f_[WL_SLOTS + 1]{};
  uint8_t cutoff_req_h_[WL_SLOTS + 1]{};
  // A queued tune open/keepalive/close: -1 none, otherwise slot * 2 + (on ? 1 : 0).
  std::atomic<int8_t> tune_req_{-1};
  std::atomic<bool> req_pending_{false};

  /* ---- the live tune session ---- */
  // The slot being tuned, -1 when none. Atomic because the log callback reads it from whichever
  // task produced the line (the two lines it parses happen to come from the main loop, but the
  // guard must be safe for every line that merely shares the tag).
  std::atomic<int8_t> tune_slot_{-1};
  /// The session's register-measured room ceiling (see TuneView::room). Atomic: written by the
  /// sampling loop on the main task, read by tune_snapshot from the httpd task.
  std::atomic<uint8_t> tune_room_{0};
  /// A session room sample on probation before it may become tune_room_. A wake word's own onset
  /// drains in the sample just BEFORE its detection event exists (the event only lands once the
  /// mean crosses the probe floor), and booking that as "room" painted an amber dot in the same
  /// beat as the attempt's blue one (owner's report, September 22 2026). So a sub-floor sample
  /// waits ~1.6s: an event arriving inside the window proves it was the utterance and drops it;
  /// a window that stays eventless commits it. millis() clock, like the event timestamps it is
  /// compared against. Main loop only.
  uint8_t tune_pend_{0};
  uint32_t tune_pend_at_{0};
  // millis_64(): wrap-proof, like every deadline here.
  uint64_t tune_deadline_ms_{0};
  // The tuned word and the ring, shared between the log callback's writer and the endpoint's
  // reader. tune_word_ is only written at session open, under the same lock the callback compares
  // it under.
  Mutex tune_lock_;
  std::string tune_word_;
  std::vector<TuneEvent> tune_events_;
  uint32_t tune_seq_{0};

  // The published view the endpoint copies from - fixed arrays, see SlotViewStore.
  Mutex view_lock_;
  SlotViewStore view_[WL_SLOTS];
  StopView stop_view_;

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
