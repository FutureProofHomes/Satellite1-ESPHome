#include "ring_fx.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace esphome::satellite1_ring {

static const char *const TAG = "satellite1_ring";

static constexpr uint8_t BLOB_VERSION = 1;
// Flash wear: a slider drag lands one write per release, and a burst of them one save.
static constexpr uint32_t SAVE_DELAY_MS = 3000;
static constexpr uint32_t PREVIEW_MIN_MS = 500;
static constexpr uint32_t PREVIEW_MAX_MS = 20000;

static const char *const STYLE_NAMES[P_COUNT + 1] = {"classic", "calm", "aurora", "party", "minimal", "custom"};
static const char *const MOMENT_KEYS[M_COUNT] = {"wake", "listen", "think", "reply", "timer",
                                                 "ring", "vol",    "mute",  "err"};
static const char *const FX_NAMES[FX_COUNT] = {"off",    "solid",   "breathe", "pulse", "spin", "comet", "orbit",
                                               "ripple", "twinkle", "wave",    "flow",  "dot",  "arc"};
static const char *const CM_NAMES[CM_COUNT] = {"ring", "blend", "rainbow", "own", "red"};

// The support signals the ring guide can play, as control_leds lights them. A negative red keeps
// voice_assistant_leds' color (the effect paints its own); bright < 0 is a floor over the LED Ring
// light's brightness: -1 the conversation floor, -2 the alert floor, -3 the sign-in floor.
struct FixedSignal {
  const char *key;
  const char *effect;
  float r, g, b;
  float bright;
};
static const FixedSignal FIXED[] = {
    {"improv", "Twinkle", 1.0f, 0.89f, 0.71f, 0.66f},
    {"init", "Twinkle", 0.094f, 0.733f, 0.949f, 0.66f},
    {"no_ha", "Twinkle", 1.0f, 0.0f, 0.0f, 0.66f},
    {"not_ready", "Twinkle", 1.0f, 0.0f, 0.0f, 0.66f},
    {"xmos", "Flashing XMOS", -1.0f, 0.0f, 0.0f, 0.6f},
    {"xmos_done", "Success", -1.0f, 0.0f, 0.0f, 0.6f},
    {"xmos_fail", "Error", -1.0f, 0.0f, 0.0f, 0.6f},
    {"login", "Breathe", 0.012f, 0.66f, 0.96f, -3.0f},
    {"login_ok", "Success", -1.0f, 0.0f, 0.0f, 0.66f},
    {"warning", "Warning", 1.0f, 0.0f, 0.0f, -2.0f},
    {"action", "Action Button Touched", -1.0f, 0.0f, 0.0f, -2.0f},
    {"jack_in", "Jack Plugged", -1.0f, 0.0f, 0.0f, -1.0f},
    {"jack_out", "Jack Unplugged", -1.0f, 0.0f, 0.0f, -2.0f},
    {"factory", "Factory Reset Coming Up", -1.0f, 0.0f, 0.0f, 1.0f},
};

static int find_name(const char *const *names, int n, const std::string &key) {
  for (int i = 0; i < n; i++)
    if (key == names[i])
      return i;
  return -1;
}

static int moment_of(const std::string &key) { return find_name(MOMENT_KEYS, M_COUNT, key); }

static const FixedSignal *fixed_of(const std::string &key) {
  for (const auto &f : FIXED)
    if (key == f.key)
      return &f;
  return nullptr;
}

static bool same_style(const Style &a, const Style &b) {
  if (a.fx != b.fx || a.cm != b.cm || a.flags != b.flags || a.sp != b.sp || a.br != b.br || a.p != b.p)
    return false;
  if (a.cm != CM_OWN)
    return true;
  return a.n == b.n && memcmp(a.c, b.c, sizeof(a.c)) == 0;
}

// "38bdf8,a78bfa" into up to three stops.
static int parse_stops(const std::string &s, uint8_t out[3][3]) {
  int n = 0;
  size_t at = 0;
  while (at <= s.size() && n < 3) {
    size_t end = s.find(',', at);
    if (end == std::string::npos)
      end = s.size();
    std::string hex = s.substr(at, end - at);
    if (!hex.empty() && hex[0] == '#')
      hex.erase(0, 1);
    if (hex.size() != 6)
      return -1;
    char *stop = nullptr;
    unsigned long v = strtoul(hex.c_str(), &stop, 16);
    if (stop == nullptr || *stop != '\0')
      return -1;
    out[n][0] = uint8_t(v >> 16);
    out[n][1] = uint8_t(v >> 8);
    out[n][2] = uint8_t(v);
    n++;
    at = end + 1;
  }
  return n;
}

void RingStyleSelect::control(size_t index) { this->parent_->select_style(uint8_t(index)); }

void RingFx::setup() {
  memcpy(this->custom_, PRESETS[P_CLASSIC], sizeof(this->custom_));
  this->pref_ = global_preferences->make_preference<Blob>(fnv1_hash("sat1_ring_v1"));
  Blob b;
  if (this->pref_.load(&b) && b.version == BLOB_VERSION) {
    this->style_ = b.style <= STYLE_CUSTOM ? b.style : uint8_t(P_CLASSIC);
    this->base_ = b.base < P_COUNT ? b.base : uint8_t(P_CLASSIC);
    memcpy(this->custom_, b.custom, sizeof(this->custom_));
    for (int m = 0; m < M_COUNT; m++)
      clamp_style(m, this->custom_[m]);
  }
  if (this->style_select_ != nullptr)
    this->style_select_->publish_state(size_t(this->style_));
  if (this->moment_sensor_ != nullptr)
    this->moment_sensor_->publish_state(this->moment_);
}

void RingFx::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Satellite1 LED ring styles:\n"
                "  Style: %s (from %s)",
                STYLE_NAMES[this->style_], STYLE_NAMES[this->base_]);
}

void RingFx::loop() {
  bool start = false, stop = false, publish = false, save = false;
  uint32_t ms = 0;
  uint8_t style = 0;
  {
    LockGuard guard{this->lock_};
    std::swap(start, this->pv_start_);
    std::swap(stop, this->pv_stop_);
    std::swap(publish, this->publish_style_);
    std::swap(save, this->save_pending_);
    ms = this->pv_ms_;
    style = this->style_;
  }
  const uint32_t now = millis();
  bool rerun = false;
  if (start) {
    this->previewing_ = true;
    this->pv_until_ = now + ms;
    rerun = true;
  } else if (stop && this->previewing_) {
    this->previewing_ = false;
    rerun = true;
  }
  if (this->previewing_ && int32_t(now - this->pv_until_) >= 0) {
    this->previewing_ = false;
    rerun = true;
  }
  // A preview takes the LED Ring light's brightness when control_leds lights it; running it again
  // when the brightness changes keeps the animation going (same effect, same moment) at the new one.
  const float bright = this->light_ != nullptr ? this->light_->remote_values.get_brightness() : 0.0f;
  if (this->previewing_ && bright != this->pv_bright_)
    rerun = true;
  this->pv_bright_ = bright;
  if (rerun)
    this->preview_callback_.call();
  if (publish && this->style_select_ != nullptr)
    this->style_select_->publish_state(size_t(style));
  if (save)
    this->schedule_save_();
  if (this->save_at_ != 0 && int32_t(now - this->save_at_) >= 0)
    this->save_();
}

void RingFx::schedule_save_() { this->save_at_ = millis() + SAVE_DELAY_MS; }

void RingFx::save_() {
  this->save_at_ = 0;
  Blob b{};
  {
    LockGuard guard{this->lock_};
    b.version = BLOB_VERSION;
    b.style = this->style_;
    b.base = this->base_;
    memcpy(b.custom, this->custom_, sizeof(b.custom));
  }
  if (!this->pref_.save(&b))
    ESP_LOGW(TAG, "Could not save the ring styles");
}

const Style &RingFx::style_for_(uint8_t m) const {
  return this->style_ == STYLE_CUSTOM ? this->custom_[m] : PRESETS[this->style_][m];
}

void RingFx::set_moment(const char *key) {
  std::string k(key);
  if (k == this->moment_)
    return;
  {
    LockGuard guard{this->lock_};
    // A real moment from control_leds outranks a preview (it only runs above the preview branch).
    if (this->previewing_ && k != this->pv_key_)
      this->previewing_ = false;
    this->moment_ = k;
  }
  this->moment_index_ = int8_t(moment_of(k));
  this->t0_ = millis();
  this->head0_ = this->head_;
  ESP_LOGV(TAG, "Moment: %s", key);
  if (this->moment_sensor_ != nullptr)
    this->moment_sensor_->publish_state(k);
}

Inputs RingFx::inputs_() {
  Inputs in;
  if (this->light_ != nullptr) {
    const auto &cv = this->light_->current_values;
    in.ring[0] = uint8_t(cv.get_red() * 255);
    in.ring[1] = uint8_t(cv.get_green() * 255);
    in.ring[2] = uint8_t(cv.get_blue() * 255);
    in.ring_on = cv.is_on();
  }
  if (this->moment_index_ == M_TIMER && this->timer_ratio_)
    in.ratio = this->timer_ratio_();
  else if (this->moment_index_ == M_VOL && this->volume_)
    in.ratio = this->volume_();
  if (this->mic_muted_)
    in.mic_muted = this->mic_muted_();
  if (this->speaker_silent_)
    in.spk_silent = this->speaker_silent_();
  // A preview shows the moment's marks and a sample arc even when nothing is muted or running.
  if (this->previewing_) {
    if (this->moment_index_ == M_TIMER)
      in.ratio = 0.62f;
    if (this->moment_index_ == M_VOL && in.ratio <= 0.0f)
      in.ratio = 0.45f;
    if (this->moment_index_ == M_MUTE)
      in.mic_muted = true;
  }
  return in;
}

void RingFx::draw(light::AddressableLight &it, bool initial_run) {
  const int8_t m = this->moment_index_;
  if (m < 0) {
    it.all() = Color::BLACK;
    return;
  }
  Inputs in = this->inputs_();
  this->drawn_ratio_ = in.ratio;
  this->drawn_mic_ = in.mic_muted;
  this->drawn_spk_ = in.spk_silent;
  Style s;
  {
    LockGuard guard{this->lock_};
    s = (this->previewing_ && this->pv_has_style_ && moment_of(this->pv_key_) == m) ? this->pv_style_
                                                                                    : this->style_for_(m);
  }
  Frame f;
  render_moment(m, s, millis() - this->t0_, this->head0_, in, f, &this->head_);
  for (int i = 0; i < N; i++)
    it[i] = Color(f[i][0], f[i][1], f[i][2]);
}

void RingFx::apply_preview(light::LightCall &call) {
  std::string key;
  {
    LockGuard guard{this->lock_};
    key = this->pv_key_;
  }
  float b = this->light_ != nullptr ? this->light_->remote_values.get_brightness() : 0.66f;
  const float conv = std::max(b, 0.2f), alert = std::min(std::max(b, 0.2f) + 0.1f, 1.0f);
  this->set_moment(key.c_str());
  call.set_state(true);
  call.set_transition_length(0);
  const int m = moment_of(key);
  if (m >= 0) {
    call.set_brightness(m <= M_REPLY ? conv : alert);
    call.set_effect("Styled");
    return;
  }
  const FixedSignal *f = fixed_of(key);
  if (f == nullptr)
    return;
  if (f->r >= 0.0f)
    call.set_rgb(f->r, f->g, f->b);
  call.set_brightness(f->bright == -1.0f   ? conv
                      : f->bright == -2.0f ? alert
                      : f->bright == -3.0f ? std::max(b, 0.25f)
                                           : f->bright);
  call.set_effect(f->effect);
}

void RingFx::select_style(uint8_t style) {
  if (style > STYLE_CUSTOM)
    return;
  {
    LockGuard guard{this->lock_};
    this->style_ = style;
  }
  if (this->style_select_ != nullptr)
    this->style_select_->publish_state(size_t(style));
  this->schedule_save_();
}

bool RingFx::patch_style_(Style &s, uint8_t m, const std::string &fx, const std::string &cm, const std::string &colors,
                          int sp, int br, int dir, int p) {
  if (!fx.empty()) {
    int i = find_name(FX_NAMES, FX_COUNT, fx);
    if (i < 0)
      return false;
    s.fx = uint8_t(i);
  }
  if (!cm.empty()) {
    int i = find_name(CM_NAMES, CM_COUNT, cm);
    if (i < 0)
      return false;
    s.cm = uint8_t(i);
  }
  if (!colors.empty()) {
    uint8_t stops[3][3] = {};
    int n = parse_stops(colors, stops);
    if (n < 1)
      return false;
    s.n = uint8_t(n);
    memcpy(s.c, stops, sizeof(s.c));
  }
  if (sp >= 0)
    s.sp = uint16_t(sp > 400 ? 400 : sp);
  if (br >= 0)
    s.br = uint8_t(br > 100 ? 100 : br);
  if (dir > 0)
    s.flags &= ~F_REV;
  else if (dir < 0)
    s.flags |= F_REV;
  if (p >= 0)
    s.p = uint8_t(p > 24 ? 24 : p);
  clamp_style(m, s);
  return true;
}

bool RingFx::web_set_style(const std::string &name) {
  int i = find_name(STYLE_NAMES, P_COUNT + 1, name);
  if (i < 0)
    return false;
  LockGuard guard{this->lock_};
  this->style_ = uint8_t(i);
  this->publish_style_ = true;
  this->save_pending_ = true;
  return true;
}

bool RingFx::web_set_moment(const std::string &mk, const std::string &fx, const std::string &cm,
                            const std::string &colors, int sp, int br, int dir, int p) {
  const int m = moment_of(mk);
  if (m < 0)
    return false;
  LockGuard guard{this->lock_};
  Style s = this->style_for_(m);
  if (!patch_style_(s, m, fx, cm, colors, sp, br, dir, p))
    return false;
  if (this->style_ != STYLE_CUSTOM) {
    memcpy(this->custom_, PRESETS[this->style_], sizeof(this->custom_));
    this->base_ = this->style_;
    this->style_ = STYLE_CUSTOM;
    this->publish_style_ = true;
  }
  this->custom_[m] = s;
  this->save_pending_ = true;
  return true;
}

bool RingFx::web_reset(const std::string &mk, bool all) {
  const int m = all ? -1 : moment_of(mk);
  if (!all && m < 0)
    return false;
  LockGuard guard{this->lock_};
  if (this->style_ != STYLE_CUSTOM)
    return true;
  if (m >= 0)
    this->custom_[m] = PRESETS[this->base_][m];
  bool same = true;
  for (int i = 0; i < M_COUNT && same; i++)
    same = same_style(this->custom_[i], PRESETS[this->base_][i]);
  // Nothing left that differs from the style it came from: back to that style itself.
  if (all || same) {
    memcpy(this->custom_, PRESETS[this->base_], sizeof(this->custom_));
    this->style_ = this->base_;
    this->publish_style_ = true;
  }
  this->save_pending_ = true;
  return true;
}

bool RingFx::web_preview(const std::string &key, uint32_t ms, const std::string &fx, const std::string &cm,
                         const std::string &colors, int sp, int br, int dir, int p) {
  const int m = moment_of(key);
  if (m < 0 && fixed_of(key) == nullptr)
    return false;
  LockGuard guard{this->lock_};
  this->pv_has_style_ = false;
  if (m >= 0 && (!fx.empty() || !cm.empty() || !colors.empty() || sp >= 0 || br >= 0 || dir != 0 || p >= 0)) {
    Style s = this->style_for_(m);
    if (!patch_style_(s, m, fx, cm, colors, sp, br, dir, p))
      return false;
    this->pv_style_ = s;
    this->pv_has_style_ = true;
  }
  this->pv_key_ = key;
  this->pv_ms_ = ms < PREVIEW_MIN_MS ? PREVIEW_MIN_MS : ms > PREVIEW_MAX_MS ? PREVIEW_MAX_MS : ms;
  this->pv_start_ = true;
  return true;
}

void RingFx::web_preview_stop() {
  LockGuard guard{this->lock_};
  this->pv_stop_ = true;
}

std::string RingFx::web_json() {
  std::string out;
  out.reserve(1100);
  char buf[160];
  LockGuard guard{this->lock_};
  snprintf(buf, sizeof(buf), R"({"style":"%s","base":"%s","moment":"%s","preview":"%s",)", STYLE_NAMES[this->style_],
           STYLE_NAMES[this->base_], this->moment_.c_str(), this->previewing_ ? this->pv_key_.c_str() : "");
  out += buf;
  snprintf(buf, sizeof(buf), R"("in":{"ratio":%d,"mic":%d,"spk":%d},"m":{)", int(this->drawn_ratio_ * 10000.0f + 0.5f),
           this->drawn_mic_ ? 1 : 0, this->drawn_spk_ ? 1 : 0);
  out += buf;
  for (int m = 0; m < M_COUNT; m++) {
    const Style &s = this->style_for_(m);
    snprintf(buf, sizeof(buf), R"(%s"%s":{"fx":"%s","cm":"%s","sp":%u,"br":%u,"fl":%u,"p":%u,"dir":%d,"c":[)",
             m ? "," : "", MOMENT_KEYS[m], FX_NAMES[s.fx < FX_COUNT ? s.fx : 0], CM_NAMES[s.cm < CM_COUNT ? s.cm : 0],
             unsigned(s.sp), unsigned(s.br), unsigned(s.flags), unsigned(s.p), (s.flags & F_REV) ? -1 : 1);
    out += buf;
    if (s.cm == CM_OWN) {
      for (int i = 0; i < s.n && i < 3; i++) {
        snprintf(buf, sizeof(buf), R"(%s"%02x%02x%02x")", i ? "," : "", s.c[i][0], s.c[i][1], s.c[i][2]);
        out += buf;
      }
    }
    out += "]}";
  }
  out += "}}";
  return out;
}

}  // namespace esphome::satellite1_ring
