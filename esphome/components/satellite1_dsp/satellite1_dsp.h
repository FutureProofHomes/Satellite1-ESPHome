#pragma once

// The speaker EQ's settings: one profile per output (the built-in speaker and the line-out jack),
// the presets they load from, and the on/off switch. The XMOS designs and runs the filters (see
// Satellite1-XMOS src/speaker_eq/); this component is the single source of truth for what it plays.
// It keeps everything in flash, uploads both profiles whenever an XMOS connects and a profile again
// after each edit to it, and tells the XMOS which one to play when the output switches.
//
// Support: an XMOS without the speaker EQ (an older release, or one picked in the XMOS firmware
// picker) answers GET_INFO with an error. The check runs in Satellite1's connected hook, before
// on_xmos_connected publishes the version, so the web UI that re-checks on that publish always gets
// the final answer. Nothing else is ever sent to an XMOS that failed it.
//
// Threads: the web_* calls come from the httpd task (satellite1_web_ui's /api/sat1/dsp routes). They
// change state under lock_ and leave the SPI traffic and flash writes to loop(), so the web UI never
// races the ESP32's own XMOS traffic.

#include "speaker_eq_protocol.h"

#include "esphome/components/satellite1/satellite1.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#ifdef USE_SAT1_DSP_DAC_PROXY
#include "esphome/components/satellite1/audio_dac/dac_proxy.h"
#endif

#include <atomic>
#include <string>

namespace esphome::satellite1_dsp {

static constexpr uint8_t NUM_PROFILES = 2;  // 0 = built-in speaker, 1 = line-out
// Three of the XMOS's 14 filter records carry Bass, Midrange and Treble.
static constexpr uint8_t MAX_FILTERS = eq::MAX_FILTERS - 3;
static constexpr uint8_t MAX_USER_PRESETS = 8;
static constexpr uint8_t NAME_LEN = 32;  // including the terminator

// Bass and Treble are shelves at fixed corners, and Midrange a peak about midway between them on a
// log scale; the web UI's lib/eq.js draws the same ones.
static constexpr float BASS_HZ = 120.0f;
static constexpr float MID_HZ = 1000.0f;
static constexpr float TREBLE_HZ = 8000.0f;
static constexpr float TONE_Q = 0.707f;
static constexpr float TONE_MAX_DB = 9.0f;

enum Support : uint8_t {
  SUPPORT_NO = 0,
  SUPPORT_YES = 1,
  SUPPORT_UNKNOWN = 2,  // no XMOS has connected since boot
};

// Where a profile stands with the XMOS, for the web UI.
enum Sync : uint8_t {
  SYNC_OK = 0,        // the XMOS plays what is stored
  SYNC_PENDING = 1,   // sent or about to be, not confirmed yet
  SYNC_REJECTED = 2,  // too much boost: the XMOS kept the previous design
  SYNC_OFFLINE = 3,   // no XMOS to send it to (booting, flashing, or not supported)
};

struct Filter {
  uint8_t type{eq::FILTER_NONE};
  uint8_t enabled{0};
  float freq{1000.0f};
  float gain{0.0f};
  float q{0.707f};
};

struct Profile {
  float preamp{0.0f};
  float bass{0.0f};
  float mid{0.0f};
  float treble{0.0f};
  float limiter{-1.0f};
  float attack{1.0f};
  float release{100.0f};
  uint8_t count{0};
  Filter filters[MAX_FILTERS];
  // The preset this was loaded from, "" for none. The web UI marks it modified by comparing.
  char preset[NAME_LEN]{};
};

struct Preset {
  char name[NAME_LEN]{};
  Profile profile;
};

// GET_INFO, decoded.
struct Info {
  bool valid{false};
  uint8_t api_major{0};
  uint8_t api_minor{0};
  uint8_t enabled{0};
  uint8_t profile{0};
  uint8_t bypass{0};
  uint8_t limiter_required{0};
  uint8_t slot_status[NUM_PROFILES]{};
  uint8_t slot_sections[NUM_PROFILES]{};
  float slot_max_gain[NUM_PROFILES]{};
  float slot_peak_gain[NUM_PROFILES]{};
  uint32_t design_count{0};
  uint32_t bank_seq{0};
};

// GET_STATS, decoded.
struct Stats {
  bool valid{false};
  uint32_t frames{0};
  uint16_t us_max[2]{};
  uint16_t us_avg[2]{};
  uint16_t us_last[2]{};
  uint32_t overruns{0};
  uint8_t bypass{0};
  uint8_t watchdog_trips{0};
  uint8_t limiter_pct[2]{};
  float limiter_max_gr{0.0f};
  uint32_t limiter_frames{0};
  uint32_t heap_min_free{0};
  uint32_t heap_free{0};
  uint32_t design_rejects{0};
  uint32_t design_us_last{0};
};

const char *builtin_preset_name(uint8_t index);
uint8_t builtin_preset_count();
// The profile each output starts with, and goes back to on reset.
void default_profile(uint8_t output, Profile &out);
// Clamps nothing: false when any field is outside what the XMOS accepts, or the filters need more
// than its 16 biquad sections. Normalizes the fields a filter type does not use.
bool validate_profile(Profile &p);
// Biquad sections the profile costs, Bass, Midrange and Treble included.
uint8_t profile_sections(const Profile &p);
// 1 to NAME_LEN - 1 bytes of well-formed UTF-8 with no control characters: a name that survives the
// JSON round trip, so the web UI can load or delete the preset by it.
bool valid_preset_name(const std::string &name);

class SpeakerDsp : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // Saves an edit still waiting out SAVE_DELAY_MS; the preferences syncer writes it to flash after.
  void on_safe_shutdown() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_satellite1(satellite1::Satellite1 *sat1) { this->sat1_ = sat1; }
#ifdef USE_SAT1_DSP_DAC_PROXY
  void set_dac_proxy(satellite1::DACProxy *dac) { this->dac_ = dac; }
#endif

  // ---- any thread ----

  Support support() const { return static_cast<Support>(this->support_.load(std::memory_order_relaxed)); }
  // Supported, and the XMOS is connected (not booting or being flashed).
  bool online() const { return this->online_.load(std::memory_order_relaxed); }
  // The profile playing now: 0 on the built-in speaker, 1 on the line-out jack.
  uint8_t live_output() const { return this->live_output_.load(std::memory_order_relaxed); }

  // ---- httpd task ----

  bool enabled();
  bool comparing();
  void get_profile(uint8_t output, Profile &out);
  Sync sync(uint8_t output);
  // Built-in presets first, then the user's.
  uint8_t preset_count();
  bool get_preset(uint8_t index, Preset &out, bool &builtin);
  void get_live(Info &info, Stats &stats);

  void web_set_enabled(bool on);
  // Hold-to-compare: the EQ is bypassed while on. Lapses on its own after COMPARE_HOLD_MS unless
  // renewed, so a browser that goes away mid-hold cannot leave the EQ off.
  void web_compare(bool on);
  bool web_set_profile(uint8_t output, const Profile &p);
  bool web_load_preset(uint8_t output, const std::string &name);
  // Saves an output's profile as a user preset, replacing one of the same name.
  bool web_save_preset(uint8_t output, const std::string &name);
  bool web_delete_preset(const std::string &name);
  // One step, so a failure can't leave both names or neither. Onto another user preset's name, that
  // preset is replaced, as a save over it would.
  bool web_rename_preset(const std::string &from, const std::string &to);
  bool web_reset(uint8_t output);
  // The web UI's Equalizer page is open: read the XMOS's info and stats every second for a while.
  void web_want_live();

 protected:
  struct Settings {
    uint8_t version;
    uint8_t enabled;
    Profile profiles[NUM_PROFILES];
  };
  struct UserPresets {
    uint8_t version;
    uint8_t count;
    Preset items[MAX_USER_PRESETS];
  };
  // The flash records are raw copies, and a record of another size fails to load: a layout change
  // would silently lose both profiles and every preset. A new layout needs a new key, and a load of
  // the old one to convert.
  static_assert(sizeof(Settings) == 484, "Settings layout changed: add a new preference key");
  static_assert(sizeof(UserPresets) == 2180, "UserPresets layout changed: add a new preference key");

  void on_connected_();
  bool read_info_(Info &info);
  bool read_stats_(Stats &stats);
  bool push_slot_(uint8_t output, const Profile &p);
  bool push_routing_(bool enabled, uint8_t output);
  uint8_t current_output_() const;
  // Takes the latest INFO: confirms the slots the XMOS has designed, and re-sends one it did not
  // take or has lost since (an XMOS that restarted behind the ESP32's back).
  void took_info_(const Info &info, uint32_t now);
  void transfer_failed_(uint32_t now);
  void back_off_(uint32_t now);
  int find_user_preset_(const std::string &name) const;
  // Lock held by the caller.
  void drop_user_preset_(int i);
  void edited_(uint8_t output);
  void save_(bool settings, bool presets);

  satellite1::Satellite1 *sat1_{nullptr};
#ifdef USE_SAT1_DSP_DAC_PROXY
  satellite1::DACProxy *dac_{nullptr};
#endif
  ESPPreferenceObject settings_pref_;
  ESPPreferenceObject presets_pref_;
  std::atomic<uint8_t> support_{SUPPORT_UNKNOWN};
  std::atomic<bool> online_{false};
  std::atomic<uint8_t> live_output_{0};

  // Guarded by lock_: everything the web routes read or change.
  Mutex lock_;
  bool enabled_{true};
  bool compare_{false};
  uint32_t compare_until_{0};
  Profile profiles_[NUM_PROFILES];
  UserPresets user_{};
  // Bumped by every edit; loop() sends a slot whose revision differs from the one it last sent.
  uint32_t rev_[NUM_PROFILES]{};
  uint32_t sent_rev_[NUM_PROFILES]{};
  // The XMOS has designed the last slot sent.
  bool confirmed_[NUM_PROFILES]{};
  // INFO's design count when each slot went out: a slot is designed once the count moves on.
  uint32_t sent_designs_[NUM_PROFILES]{};
  // Times the XMOS did not take the latest revision; at REFUSALS_MAX it is held until the next edit.
  uint8_t refusals_[NUM_PROFILES]{};
  bool routing_dirty_{true};
  bool settings_changed_{false};
  bool presets_changed_{false};
  bool live_{false};
  uint32_t live_until_{0};
  Info info_;
  Stats stats_;

  // Main loop only.
  uint32_t last_push_{0};
  uint32_t info_at_{0};
  uint8_t info_tries_{0};
  uint32_t live_read_at_{0};
  uint32_t heartbeat_at_{0};
  uint32_t retry_at_{0};
  uint32_t backoff_ms_{0};
  uint32_t save_at_{0};
  bool save_settings_{false};
  bool save_presets_{false};
  bool sent_enabled_{false};
  uint8_t sent_output_{0xFF};
};

}  // namespace esphome::satellite1_dsp
