#include "satellite1_dsp.h"

#include "esphome/core/log.h"

#include <algorithm>
#include <cstring>

namespace esphome::satellite1_dsp {

static const char *const TAG = "satellite1_dsp";

static constexpr uint8_t SETTINGS_VERSION = 1;
static constexpr uint8_t PRESETS_VERSION = 1;
static constexpr uint32_t SAVE_DELAY_MS = 3000;
// About ten uploads a second while a slider is dragged.
static constexpr uint32_t PUSH_INTERVAL_MS = 100;
// The XMOS designs a slot on a low-priority task, within a few ms of receiving it.
static constexpr uint32_t INFO_DELAY_MS = 30;
static constexpr uint8_t INFO_TRIES = 10;
static constexpr uint32_t LIVE_MS = 5000;
static constexpr uint32_t LIVE_READ_MS = 1000;
// Catches an XMOS that restarted without the ESP32 seeing it go (the XMOS Hardware Reset button
// pulses its reset line and leaves Satellite1 connected): its slots come back empty.
static constexpr uint32_t HEARTBEAT_MS = 10000;
static constexpr uint32_t COMPARE_HOLD_MS = 15000;
static constexpr uint32_t BACKOFF_MIN_MS = 500;
static constexpr uint32_t BACKOFF_MAX_MS = 10000;
// The ESP32 checks the same limits first, so a refusal means the two firmwares disagree.
static constexpr uint8_t REFUSALS_MAX = 3;

static const char *const OUTPUT_NAMES[NUM_PROFILES] = {"Built-in Speaker", "Line-out"};

static uint8_t profile_id(uint8_t output) { return output == 1 ? eq::PROFILE_LINE_OUT : eq::PROFILE_SPEAKER; }

// ---- Presets ----

static void make_sat1_speaker(Profile &p) {
  // The 2026-10-10 tuning, exported from Music Assistant, played 2 dB below the export's +10 dB
  // preamp. Its net response peaks at +12.8 dB at 5.7 kHz, so it still leans on the limiter.
  static const Filter FILTERS[] = {
      {eq::FILTER_PEAK, 1, 138.0f, -3.0f, 1.93f},       {eq::FILTER_PEAK, 1, 1600.0f, -4.5f, 4.0f},
      {eq::FILTER_PEAK, 1, 3455.0f, -8.0f, 2.66f},      {eq::FILTER_PEAK, 1, 5700.0f, 7.0f, 3.42f},
      {eq::FILTER_PEAK, 1, 13700.0f, -10.0f, 1.0f},     {eq::FILTER_HIGHPASS_Q, 1, 60.0f, 0.0f, 1.0f},
  };
  p = Profile{};
  p.preamp = 8.0f;
  p.count = sizeof(FILTERS) / sizeof(FILTERS[0]);
  memcpy(p.filters, FILTERS, sizeof(FILTERS));
}

static void make_flat(Profile &p) { p = Profile{}; }

struct Builtin {
  const char *name;
  void (*make)(Profile &);
};

static const Builtin BUILTINS[] = {
    {"Sat1.2 Smart Speaker (10/26)", make_sat1_speaker},
    {"Flat", make_flat},
};
static constexpr uint8_t NUM_BUILTINS = sizeof(BUILTINS) / sizeof(BUILTINS[0]);
// Which built-in each output starts with.
static constexpr uint8_t DEFAULT_PRESET[NUM_PROFILES] = {0, 1};

const char *builtin_preset_name(uint8_t index) { return index < NUM_BUILTINS ? BUILTINS[index].name : ""; }
uint8_t builtin_preset_count() { return NUM_BUILTINS; }

static void set_name(char (&dst)[NAME_LEN], const char *src) {
  strncpy(dst, src, NAME_LEN - 1);
  dst[NAME_LEN - 1] = '\0';
}

static int find_builtin(const std::string &name) {
  for (uint8_t i = 0; i < NUM_BUILTINS; i++) {
    if (name == BUILTINS[i].name)
      return i;
  }
  return -1;
}

void default_profile(uint8_t output, Profile &out) {
  const Builtin &b = BUILTINS[DEFAULT_PRESET[output < NUM_PROFILES ? output : 0]];
  b.make(out);
  set_name(out.preset, b.name);
}

// ---- Validation ----

// Comparisons are false for NaN, so this rejects NaN and infinities too.
static bool in_range(float v, float lo, float hi) { return v >= lo && v <= hi; }

uint8_t profile_sections(const Profile &p) {
  unsigned n = eq::filter_sections(eq::FILTER_LOW_SHELF, true, p.bass) +
               eq::filter_sections(eq::FILTER_PEAK, true, p.mid) +
               eq::filter_sections(eq::FILTER_HIGH_SHELF, true, p.treble);
  for (uint8_t i = 0; i < p.count && i < MAX_FILTERS; i++)
    n += eq::filter_sections(p.filters[i].type, p.filters[i].enabled != 0, p.filters[i].gain);
  return static_cast<uint8_t>(n);
}

bool validate_profile(Profile &p) {
  p.preset[NAME_LEN - 1] = '\0';
  if (!in_range(p.preamp, eq::PREAMP_MIN_DB, eq::PREAMP_MAX_DB) || !in_range(p.bass, -TONE_MAX_DB, TONE_MAX_DB) ||
      !in_range(p.mid, -TONE_MAX_DB, TONE_MAX_DB) || !in_range(p.treble, -TONE_MAX_DB, TONE_MAX_DB) ||
      !in_range(p.limiter, eq::LIMITER_MIN_DB, eq::LIMITER_MAX_DB) ||
      !in_range(p.attack, eq::ATTACK_MIN_MS, eq::ATTACK_MAX_MS) ||
      !in_range(p.release, eq::RELEASE_MIN_MS, eq::RELEASE_MAX_MS) || p.count > MAX_FILTERS) {
    return false;
  }
  for (uint8_t i = 0; i < p.count; i++) {
    Filter &f = p.filters[i];
    if (f.type == eq::FILTER_NONE || f.type >= eq::NUM_FILTER_TYPES || !in_range(f.freq, eq::FREQ_MIN_HZ, eq::FREQ_MAX_HZ))
      return false;
    f.enabled = f.enabled ? 1 : 0;
    if (eq::filter_uses_q(f.type)) {
      if (!in_range(f.q, eq::Q_MIN, eq::Q_MAX))
        return false;
    } else {
      f.q = 0.707f;
    }
    if (eq::filter_uses_gain(f.type)) {
      if (!in_range(f.gain, eq::GAIN_MIN_DB, eq::GAIN_MAX_DB))
        return false;
    } else {
      f.gain = 0.0f;
    }
  }
  for (uint8_t i = p.count; i < MAX_FILTERS; i++)
    p.filters[i] = Filter{};
  return profile_sections(p) <= eq::MAX_SECTIONS;
}

bool valid_preset_name(const std::string &name) {
  if (name.empty() || name.size() >= NAME_LEN)
    return false;
  for (size_t i = 0; i < name.size();) {
    const uint8_t c = static_cast<uint8_t>(name[i]);
    size_t more;
    if (c < 0x80) {
      if (c < 0x20 || c == 0x7F)
        return false;
      more = 0;
    } else if (c >= 0xC2 && c <= 0xDF) {
      more = 1;
    } else if (c >= 0xE0 && c <= 0xEF) {
      more = 2;
    } else if (c >= 0xF0 && c <= 0xF4) {
      more = 3;
    } else {
      return false;
    }
    if (i + more >= name.size())
      return false;
    for (size_t k = 1; k <= more; k++) {
      if ((static_cast<uint8_t>(name[i + k]) & 0xC0) != 0x80)
        return false;
    }
    if (more > 1) {
      // Overlong forms, UTF-16 surrogates, and past U+10FFFF.
      const uint8_t c1 = static_cast<uint8_t>(name[i + 1]);
      if ((c == 0xE0 && c1 < 0xA0) || (c == 0xED && c1 >= 0xA0) || (c == 0xF0 && c1 < 0x90) ||
          (c == 0xF4 && c1 >= 0x90))
        return false;
    }
    i += more + 1;
  }
  return true;
}

// ---- Wire format ----

static void put_u32(uint8_t *b, uint32_t v) {
  b[0] = v & 0xFF;
  b[1] = (v >> 8) & 0xFF;
  b[2] = (v >> 16) & 0xFF;
  b[3] = (v >> 24) & 0xFF;
}

static void put_f32(uint8_t *b, float v) {
  uint32_t u;
  memcpy(&u, &v, sizeof(u));
  put_u32(b, u);
}

static uint32_t get_u32(const uint8_t *b) {
  return uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) | (uint32_t(b[3]) << 24);
}

static uint16_t get_u16(const uint8_t *b) { return uint16_t(b[0] | (b[1] << 8)); }

static float get_f32(const uint8_t *b) {
  const uint32_t u = get_u32(b);
  float v;
  memcpy(&v, &u, sizeof(v));
  return v;
}

// ---- Component ----

void SpeakerDsp::setup() {
  for (uint8_t o = 0; o < NUM_PROFILES; o++)
    default_profile(o, this->profiles_[o]);

  this->settings_pref_ = global_preferences->make_preference<Settings>(fnv1_hash("sat1_dsp_v1"));
  Settings s{};
  if (this->settings_pref_.load(&s) && s.version == SETTINGS_VERSION) {
    this->enabled_ = s.enabled != 0;
    for (uint8_t o = 0; o < NUM_PROFILES; o++) {
      if (validate_profile(s.profiles[o])) {
        this->profiles_[o] = s.profiles[o];
      } else {
        ESP_LOGW(TAG, "Stored %s profile is invalid; using the default", OUTPUT_NAMES[o]);
      }
    }
  }

  // Straight into the member: 2.3 KB is more than belongs on the loop task's stack.
  this->presets_pref_ = global_preferences->make_preference<UserPresets>(fnv1_hash("sat1_dsp_presets_v1"));
  if (!this->presets_pref_.load(&this->user_) || this->user_.version != PRESETS_VERSION) {
    this->user_ = UserPresets{};
  } else {
    uint8_t kept = 0;
    for (uint8_t i = 0; i < this->user_.count && i < MAX_USER_PRESETS; i++) {
      Preset &p = this->user_.items[i];
      p.name[NAME_LEN - 1] = '\0';
      if (p.name[0] == '\0' || find_builtin(p.name) >= 0 || !validate_profile(p.profile))
        continue;
      if (kept != i)
        this->user_.items[kept] = p;
      kept++;
    }
    this->user_.count = kept;
  }
  this->user_.version = PRESETS_VERSION;

  this->live_output_.store(this->current_output_(), std::memory_order_relaxed);
  this->sat1_->add_on_connected_hook([this]() { this->on_connected_(); });
}

void SpeakerDsp::dump_config() {
  // Also runs when a log client subscribes, which can be mid-edit from the web UI.
  LockGuard guard{this->lock_};
  ESP_LOGCONFIG(TAG, "Satellite1 speaker EQ:");
  ESP_LOGCONFIG(TAG, "  Enabled: %s", YESNO(this->enabled_));
  for (uint8_t o = 0; o < NUM_PROFILES; o++) {
    const Profile &p = this->profiles_[o];
    ESP_LOGCONFIG(TAG, "  %s: \"%s\", %u filters, preamp %.1f dB", OUTPUT_NAMES[o], p.preset,
                  static_cast<unsigned>(p.count), p.preamp);
  }
  ESP_LOGCONFIG(TAG, "  User presets: %u", static_cast<unsigned>(this->user_.count));
#ifdef USE_SAT1_DSP_DAC_PROXY
  ESP_LOGCONFIG(TAG, "  Follows the output switch: YES");
#endif
}

uint8_t SpeakerDsp::current_output_() const {
#ifdef USE_SAT1_DSP_DAC_PROXY
  if (this->dac_ != nullptr && this->dac_->active_dac == satellite1::LINE_OUT)
    return 1;
#endif
  return 0;
}

void SpeakerDsp::on_connected_() {
  Info info;
  const bool ok = this->read_info_(info) && info.api_major == eq::API_MAJOR;
  this->support_.store(ok ? SUPPORT_YES : SUPPORT_NO, std::memory_order_relaxed);
  if (!ok) {
    ESP_LOGI(TAG, "This XMOS firmware has no speaker EQ this build can drive");
    // The last firmware's readings would otherwise outlive it on /api/sat1/dsp/live.
    LockGuard guard{this->lock_};
    this->info_ = Info{};
    this->stats_ = Stats{};
    return;
  }
  ESP_LOGI(TAG, "XMOS speaker EQ API %u.%u", info.api_major, info.api_minor);

  // A fresh XMOS has nothing: send both profiles, then which one to play.
  LockGuard guard{this->lock_};
  this->info_ = info;
  for (uint8_t o = 0; o < NUM_PROFILES; o++) {
    this->sent_rev_[o] = this->rev_[o] - 1;
    this->confirmed_[o] = false;
    this->refusals_[o] = 0;
  }
  this->routing_dirty_ = true;
  this->sent_output_ = 0xFF;
  this->info_at_ = 0;
  this->retry_at_ = 0;
  this->backoff_ms_ = 0;
  this->heartbeat_at_ = millis() + HEARTBEAT_MS;
}

bool SpeakerDsp::read_info_(Info &info) {
  uint8_t b[eq::INFO_BYTES];
  if (!this->sat1_->transfer(eq::RESID, eq::CMD_GET_INFO | satellite1::CONTROL_CMD_READ_BIT, b, sizeof(b)))
    return false;
  info.valid = true;
  info.api_major = b[0];
  info.api_minor = b[1];
  info.enabled = b[8];
  info.profile = b[9];
  info.bypass = b[10];
  info.limiter_required = b[11];
  for (uint8_t s = 0; s < NUM_PROFILES; s++) {
    info.slot_status[s] = b[12 + s];
    info.slot_sections[s] = b[14 + s];
    info.slot_max_gain[s] = get_f32(&b[16 + 4 * s]);
    info.slot_peak_gain[s] = get_f32(&b[24 + 4 * s]);
  }
  info.design_count = get_u32(&b[32]);
  info.bank_seq = get_u32(&b[36]);
  return true;
}

bool SpeakerDsp::read_stats_(Stats &stats) {
  uint8_t b[eq::STATS_BYTES];
  if (!this->sat1_->transfer(eq::RESID, eq::CMD_GET_STATS | satellite1::CONTROL_CMD_READ_BIT, b, sizeof(b)))
    return false;
  stats.valid = true;
  stats.frames = get_u32(&b[0]);
  for (uint8_t c = 0; c < 2; c++) {
    stats.us_max[c] = get_u16(&b[4 + 2 * c]);
    stats.us_avg[c] = get_u16(&b[8 + 2 * c]);
    stats.us_last[c] = get_u16(&b[12 + 2 * c]);
    stats.limiter_pct[c] = b[22 + c];
  }
  stats.overruns = get_u32(&b[16]);
  stats.bypass = b[20];
  stats.watchdog_trips = b[21];
  stats.limiter_max_gr = get_f32(&b[24]);
  stats.limiter_frames = get_u32(&b[28]);
  stats.heap_min_free = get_u32(&b[32]);
  stats.heap_free = get_u32(&b[36]);
  stats.design_rejects = get_u32(&b[40]);
  stats.design_us_last = get_u32(&b[44]);
  return true;
}

bool SpeakerDsp::push_slot_(uint8_t output, const Profile &p) {
  uint8_t b[eq::SLOT_BYTES] = {0};
  uint8_t n = 0;
  // Bass, Midrange and Treble take the first three records, so adding or removing a filter does not
  // renumber them and the XMOS can glide them rather than crossfade.
  auto put_filter = [&b, &n](uint8_t type, bool enabled, float freq, float gain, float q) {
    uint8_t *r = &b[eq::SLOT_HEADER_BYTES + n * eq::FILTER_RECORD_BYTES];
    r[0] = type;
    r[1] = enabled ? eq::FILTER_FLAG_ENABLED : 0;
    put_f32(&r[2], freq);
    put_f32(&r[6], gain);
    put_f32(&r[10], q);
    n++;
  };

  b[0] = profile_id(output);
  put_f32(&b[1], p.preamp);
  put_f32(&b[5], p.limiter);
  put_f32(&b[9], p.attack);
  put_f32(&b[13], p.release);
  put_filter(eq::FILTER_LOW_SHELF, true, BASS_HZ, p.bass, TONE_Q);
  put_filter(eq::FILTER_PEAK, true, MID_HZ, p.mid, TONE_Q);
  put_filter(eq::FILTER_HIGH_SHELF, true, TREBLE_HZ, p.treble, TONE_Q);
  for (uint8_t i = 0; i < p.count && i < MAX_FILTERS; i++) {
    const Filter &f = p.filters[i];
    put_filter(f.type, f.enabled != 0, f.freq, f.gain, f.q);
  }
  b[17] = n;
  return this->sat1_->transfer(eq::RESID, eq::CMD_SET_SLOT, b, sizeof(b));
}

bool SpeakerDsp::push_routing_(bool enabled, uint8_t output) {
  uint8_t b[eq::ROUTING_BYTES] = {static_cast<uint8_t>(enabled ? 1 : 0), profile_id(output)};
  return this->sat1_->transfer(eq::RESID, eq::CMD_SET_ROUTING, b, sizeof(b));
}

void SpeakerDsp::transfer_failed_(uint32_t now) {
  if (this->backoff_ms_ == 0)
    ESP_LOGW(TAG, "XMOS did not take a speaker EQ command; retrying");
  this->back_off_(now);
}

void SpeakerDsp::back_off_(uint32_t now) {
  this->backoff_ms_ = this->backoff_ms_ == 0 ? BACKOFF_MIN_MS : std::min(this->backoff_ms_ * 2, BACKOFF_MAX_MS);
  this->retry_at_ = now + this->backoff_ms_;
}

void SpeakerDsp::took_info_(const Info &info, uint32_t now) {
  LockGuard guard{this->lock_};
  this->info_ = info;
  for (uint8_t o = 0; o < NUM_PROFILES; o++) {
    const uint8_t status = info.slot_status[o];
    // A slot with an edit still to send is sent regardless; one being designed has no answer yet.
    if (this->sent_rev_[o] != this->rev_[o] || status == eq::SLOT_PENDING)
      continue;
    if (this->confirmed_[o]) {
      if (status == eq::SLOT_EMPTY) {
        ESP_LOGW(TAG, "XMOS lost the %s profile; sending it again", OUTPUT_NAMES[o]);
        this->sent_rev_[o] = this->rev_[o] - 1;
        this->confirmed_[o] = false;
      }
      continue;
    }
    if (this->refusals_[o] >= REFUSALS_MAX)
      continue;
    // A slot the XMOS accepted reads PENDING until it is designed, and the design moves the count
    // on. Neither, and the write never took.
    if (status != eq::SLOT_EMPTY && info.design_count != this->sent_designs_[o]) {
      this->confirmed_[o] = true;
      continue;
    }
    if (++this->refusals_[o] < REFUSALS_MAX) {
      ESP_LOGW(TAG, "XMOS did not take the %s profile; sending it again", OUTPUT_NAMES[o]);
      this->sent_rev_[o] = this->rev_[o] - 1;
      this->back_off_(now);
    } else {
      ESP_LOGE(TAG, "XMOS refused the %s profile %u times; holding it until the next edit", OUTPUT_NAMES[o],
               static_cast<unsigned>(REFUSALS_MAX));
    }
  }
  if (!this->routing_dirty_ && this->sent_output_ < NUM_PROFILES &&
      (info.enabled != (this->sent_enabled_ ? 1 : 0) || info.profile != profile_id(this->sent_output_))) {
    this->routing_dirty_ = true;
  }
}

void SpeakerDsp::loop() {
  const uint32_t now = millis();

  bool live = false;
  {
    LockGuard guard{this->lock_};
    if (this->compare_ && static_cast<int32_t>(now - this->compare_until_) >= 0) {
      this->compare_ = false;
      this->routing_dirty_ = true;
    }
    const uint8_t output = this->current_output_();
    if (output != this->live_output_.load(std::memory_order_relaxed)) {
      this->live_output_.store(output, std::memory_order_relaxed);
      this->routing_dirty_ = true;
    }
    if (this->settings_changed_ || this->presets_changed_) {
      this->save_settings_ |= this->settings_changed_;
      this->save_presets_ |= this->presets_changed_;
      this->settings_changed_ = this->presets_changed_ = false;
      this->save_at_ = now + SAVE_DELAY_MS;
    }
    // Deadlines are cleared once passed: a stale one compared 2^31 ms later reads as the future.
    if (this->live_ && static_cast<int32_t>(now - this->live_until_) >= 0)
      this->live_ = false;
    live = this->live_;
  }
  if (this->save_at_ != 0 && static_cast<int32_t>(now - this->save_at_) >= 0) {
    this->save_(this->save_settings_, this->save_presets_);
    this->save_at_ = 0;
    this->save_settings_ = this->save_presets_ = false;
  }

  const bool connected = this->sat1_->is_xmos_connected();
  const bool online = this->support() == SUPPORT_YES && connected;
  this->online_.store(online, std::memory_order_relaxed);
  if (!online)
    return;
  if (this->retry_at_ != 0) {
    if (static_cast<int32_t>(now - this->retry_at_) < 0)
      return;
    this->retry_at_ = 0;
  }

  // One write per pass: a slot that changed, else the routing. Slots go first so that a fresh XMOS
  // has its designs before it is told which one to play. Nothing goes out while the last slot sent
  // is being confirmed, so the design count it is checked against is exact.
  if (this->info_at_ == 0 && now - this->last_push_ >= PUSH_INTERVAL_MS) {
    Profile p;
    int output = -1;
    uint32_t rev = 0;
    bool routing = false;
    bool enabled = false;
    const uint8_t live_output = this->live_output_.load(std::memory_order_relaxed);
    {
      LockGuard guard{this->lock_};
      for (uint8_t o = 0; o < NUM_PROFILES && output < 0; o++) {
        if (this->sent_rev_[o] != this->rev_[o]) {
          output = o;
          p = this->profiles_[o];
          rev = this->rev_[o];
        }
      }
      if (output < 0 && this->routing_dirty_) {
        routing = true;
        enabled = this->enabled_ && !this->compare_;
        this->routing_dirty_ = false;
      }
    }
    if (output >= 0 || routing) {
      this->last_push_ = now;
      const bool ok = output >= 0 ? this->push_slot_(output, p) : this->push_routing_(enabled, live_output);
      LockGuard guard{this->lock_};
      if (!ok) {
        if (routing)
          this->routing_dirty_ = true;
        this->transfer_failed_(now);
        return;
      }
      this->backoff_ms_ = 0;
      if (output >= 0) {
        this->sent_rev_[output] = rev;
        this->confirmed_[output] = false;
        this->sent_designs_[output] = this->info_.design_count;
        this->info_at_ = now + INFO_DELAY_MS;
        this->info_tries_ = 0;
      } else {
        this->sent_enabled_ = enabled;
        this->sent_output_ = live_output;
      }
      return;
    }
  }

  // After an upload, read INFO until the XMOS has designed it. A failed read stays armed and is
  // retried after the backoff; one still pending after INFO_TRIES is left to the heartbeat.
  if (this->info_at_ != 0) {
    if (static_cast<int32_t>(now - this->info_at_) < 0)
      return;
    Info info;
    if (!this->read_info_(info)) {
      this->transfer_failed_(now);
      return;
    }
    this->backoff_ms_ = 0;
    this->took_info_(info, now);
    const bool pending =
        info.slot_status[0] == eq::SLOT_PENDING || info.slot_status[1] == eq::SLOT_PENDING;
    this->info_at_ = pending && ++this->info_tries_ < INFO_TRIES ? now + INFO_DELAY_MS : 0;
    this->heartbeat_at_ = now + HEARTBEAT_MS;
    return;
  }

  // Info and stats every second while the Equalizer is open, INFO alone every HEARTBEAT_MS otherwise.
  const bool live_due = live && now - this->live_read_at_ >= LIVE_READ_MS;
  if (live_due || static_cast<int32_t>(now - this->heartbeat_at_) >= 0) {
    Info info;
    Stats stats;
    if (!this->read_info_(info) || (live_due && !this->read_stats_(stats))) {
      this->heartbeat_at_ = now + HEARTBEAT_MS;
      this->transfer_failed_(now);
      return;
    }
    this->backoff_ms_ = 0;
    this->took_info_(info, now);
    if (live_due) {
      LockGuard guard{this->lock_};
      this->stats_ = stats;
      this->live_read_at_ = now;
    }
    this->heartbeat_at_ = now + HEARTBEAT_MS;
  }
}

void SpeakerDsp::save_(bool settings, bool presets) {
  if (settings) {
    Settings s{};
    {
      LockGuard guard{this->lock_};
      s.version = SETTINGS_VERSION;
      s.enabled = this->enabled_ ? 1 : 0;
      memcpy(s.profiles, this->profiles_, sizeof(s.profiles));
    }
    if (!this->settings_pref_.save(&s))
      ESP_LOGW(TAG, "Could not save the speaker EQ");
  }
  if (presets) {
    // Saved from the member under the lock rather than through a 2.3 KB copy on this stack.
    LockGuard guard{this->lock_};
    if (!this->presets_pref_.save(&this->user_))
      ESP_LOGW(TAG, "Could not save the speaker EQ presets");
  }
  // save() only queues the record; the preferences syncer would write it up to a minute later.
  if (settings || presets)
    global_preferences->sync();
}

void SpeakerDsp::on_safe_shutdown() {
  {
    LockGuard guard{this->lock_};
    this->save_settings_ |= this->settings_changed_;
    this->save_presets_ |= this->presets_changed_;
  }
  this->save_(this->save_settings_, this->save_presets_);
}

// ---- httpd task ----

bool SpeakerDsp::enabled() {
  LockGuard guard{this->lock_};
  return this->enabled_;
}

bool SpeakerDsp::comparing() {
  LockGuard guard{this->lock_};
  return this->compare_;
}

void SpeakerDsp::get_profile(uint8_t output, Profile &out) {
  LockGuard guard{this->lock_};
  out = this->profiles_[output < NUM_PROFILES ? output : 0];
}

Sync SpeakerDsp::sync(uint8_t output) {
  if (output >= NUM_PROFILES || !this->online())
    return SYNC_OFFLINE;
  LockGuard guard{this->lock_};
  if (this->sent_rev_[output] != this->rev_[output])
    return SYNC_PENDING;
  if (this->refusals_[output] >= REFUSALS_MAX)
    return SYNC_REJECTED;
  if (!this->confirmed_[output])
    return SYNC_PENDING;
  if (this->info_.valid && this->info_.slot_status[output] == eq::SLOT_REJECTED_HEADROOM)
    return SYNC_REJECTED;
  return SYNC_OK;
}

uint8_t SpeakerDsp::preset_count() {
  LockGuard guard{this->lock_};
  return NUM_BUILTINS + this->user_.count;
}

bool SpeakerDsp::get_preset(uint8_t index, Preset &out, bool &builtin) {
  if (index < NUM_BUILTINS) {
    builtin = true;
    set_name(out.name, BUILTINS[index].name);
    BUILTINS[index].make(out.profile);
    set_name(out.profile.preset, BUILTINS[index].name);
    return true;
  }
  builtin = false;
  LockGuard guard{this->lock_};
  const uint8_t i = index - NUM_BUILTINS;
  if (i >= this->user_.count)
    return false;
  out = this->user_.items[i];
  return true;
}

void SpeakerDsp::get_live(Info &info, Stats &stats) {
  LockGuard guard{this->lock_};
  info = this->info_;
  stats = this->stats_;
}

void SpeakerDsp::web_set_enabled(bool on) {
  LockGuard guard{this->lock_};
  if (this->enabled_ == on)
    return;
  this->enabled_ = on;
  this->routing_dirty_ = true;
  this->settings_changed_ = true;
}

void SpeakerDsp::web_compare(bool on) {
  LockGuard guard{this->lock_};
  this->compare_until_ = millis() + COMPARE_HOLD_MS;
  if (this->compare_ == on)
    return;
  this->compare_ = on;
  this->routing_dirty_ = true;
}

void SpeakerDsp::edited_(uint8_t output) {
  this->rev_[output]++;
  this->confirmed_[output] = false;
  this->refusals_[output] = 0;
  this->settings_changed_ = true;
}

bool SpeakerDsp::web_set_profile(uint8_t output, const Profile &p) {
  if (output >= NUM_PROFILES)
    return false;
  Profile v = p;
  if (!validate_profile(v))
    return false;
  LockGuard guard{this->lock_};
  this->profiles_[output] = v;
  this->edited_(output);
  return true;
}

int SpeakerDsp::find_user_preset_(const std::string &name) const {
  for (uint8_t i = 0; i < this->user_.count; i++) {
    if (name == this->user_.items[i].name)
      return i;
  }
  return -1;
}

bool SpeakerDsp::web_load_preset(uint8_t output, const std::string &name) {
  if (output >= NUM_PROFILES)
    return false;
  Profile p;
  const int b = find_builtin(name);
  LockGuard guard{this->lock_};
  if (b >= 0) {
    BUILTINS[b].make(p);
  } else {
    const int u = this->find_user_preset_(name);
    if (u < 0)
      return false;
    p = this->user_.items[u].profile;
  }
  set_name(p.preset, name.c_str());
  this->profiles_[output] = p;
  this->edited_(output);
  return true;
}

bool SpeakerDsp::web_save_preset(uint8_t output, const std::string &name) {
  if (output >= NUM_PROFILES || !valid_preset_name(name) || find_builtin(name) >= 0)
    return false;
  LockGuard guard{this->lock_};
  int i = this->find_user_preset_(name);
  if (i < 0) {
    if (this->user_.count >= MAX_USER_PRESETS)
      return false;
    i = this->user_.count++;
  }
  Preset &dst = this->user_.items[i];
  set_name(dst.name, name.c_str());
  set_name(this->profiles_[output].preset, name.c_str());
  dst.profile = this->profiles_[output];
  this->presets_changed_ = true;
  this->settings_changed_ = true;
  return true;
}

void SpeakerDsp::drop_user_preset_(int i) {
  for (uint8_t k = i; k + 1 < this->user_.count; k++)
    this->user_.items[k] = this->user_.items[k + 1];
  this->user_.count--;
  this->user_.items[this->user_.count] = Preset{};
  this->presets_changed_ = true;
}

bool SpeakerDsp::web_delete_preset(const std::string &name) {
  LockGuard guard{this->lock_};
  const int i = this->find_user_preset_(name);
  if (i < 0)
    return false;
  this->drop_user_preset_(i);
  // A profile loaded from it keeps its sound but no longer names a preset that does not exist.
  for (auto &p : this->profiles_) {
    if (name == p.preset) {
      p.preset[0] = '\0';
      this->settings_changed_ = true;
    }
  }
  return true;
}

bool SpeakerDsp::web_rename_preset(const std::string &from, const std::string &to) {
  if (!valid_preset_name(to) || find_builtin(to) >= 0)
    return false;
  LockGuard guard{this->lock_};
  int i = this->find_user_preset_(from);
  if (i < 0)
    return false;
  if (from == to)
    return true;
  const int clash = this->find_user_preset_(to);
  if (clash >= 0) {
    this->drop_user_preset_(clash);
    if (clash < i)
      i--;
  }
  Preset &dst = this->user_.items[i];
  set_name(dst.name, to.c_str());
  set_name(dst.profile.preset, to.c_str());
  this->presets_changed_ = true;
  // Outputs on the renamed preset follow it; any on the one it replaced become Custom, as if that
  // one had been deleted.
  for (auto &p : this->profiles_) {
    if (clash >= 0 && to == p.preset) {
      p.preset[0] = '\0';
      this->settings_changed_ = true;
    } else if (from == p.preset) {
      set_name(p.preset, to.c_str());
      this->settings_changed_ = true;
    }
  }
  return true;
}

bool SpeakerDsp::web_reset(uint8_t output) {
  if (output >= NUM_PROFILES)
    return false;
  Profile p;
  default_profile(output, p);
  LockGuard guard{this->lock_};
  this->profiles_[output] = p;
  this->edited_(output);
  return true;
}

void SpeakerDsp::web_want_live() {
  LockGuard guard{this->lock_};
  this->live_ = true;
  this->live_until_ = millis() + LIVE_MS;
}

}  // namespace esphome::satellite1_dsp
