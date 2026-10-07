#pragma once

// The LED ring's styles: which animation each moment of the device's life plays (wake word heard,
// listening, thinking, replying, timers, volume, muted, errors), stored in flash, chosen from Home
// Assistant through the "LED Ring Style" select and edited from the web UI's LED Ring page.
//
// The ring itself is two partition lights over the same 24 LEDs (config/common/led_ring.yaml):
// led_ring, the customer's light, and voice_assistant_leds, which control_leds drives. Each
// control_leds script that plays a styled moment calls set_moment() and starts the "Styled" effect
// on voice_assistant_leds, whose lambda calls draw() every frame. While the device is idle the ring
// is off, or led_ring's plain color when the customer has that light on.
//
// Threads: set_moment, draw, apply_preview and the select run on the main loop. The web_* calls
// come from the httpd task (satellite1_web_ui's /api/sat1/ring routes); they change state under
// lock_ and leave anything that has to happen on the main loop (saving, publishing, re-running
// control_leds) as a flag loop() picks up.

#include "ring_fx_core.h"

#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include <functional>
#include <string>

namespace esphome::satellite1_ring {

// Index 5 of the select, after the five presets.
static constexpr uint8_t STYLE_CUSTOM = P_COUNT;

class RingFx;

class RingStyleSelect : public select::Select, public Parented<RingFx> {
 protected:
  void control(size_t index) override;
};

class RingFx : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_light(light::LightState *light) { this->light_ = light; }
  void set_timer_ratio(std::function<float()> &&f) { this->timer_ratio_ = std::move(f); }
  void set_volume(std::function<float()> &&f) { this->volume_ = std::move(f); }
  void set_mic_muted(std::function<bool()> &&f) { this->mic_muted_ = std::move(f); }
  void set_speaker_silent(std::function<bool()> &&f) { this->speaker_silent_ = std::move(f); }
  void set_style_select(RingStyleSelect *s) { this->style_select_ = s; }
  void set_moment_sensor(text_sensor::TextSensor *s) { this->moment_sensor_ = s; }
  void add_on_preview_callback(std::function<void()> &&cb) { this->preview_callback_.add(std::move(cb)); }

  // ---- Main loop ----

  // What the ring is showing: a style moment ("wake" ... "err"), "idle", or a fixed signal's name.
  void set_moment(const char *key);
  // True while a web preview owns the ring; control_leds checks it below the voice phases and
  // alerts, and then runs a script that hands a LightCall to apply_preview().
  bool is_previewing() const { return this->previewing_; }
  void apply_preview(light::LightCall &call);
  // The Styled effect's frame.
  void draw(light::AddressableLight &it, bool initial_run);
  void select_style(uint8_t style);

  // ---- httpd task ----

  bool web_set_style(const std::string &name);
  // One moment's style; empty strings leave a field as it is. Editing a preset copies it into
  // Custom first.
  bool web_set_moment(const std::string &m, const std::string &fx, const std::string &cm, const std::string &colors,
                      int sp, int br, int dir, int p);
  bool web_reset(const std::string &m, bool all);
  // Plays a moment or fixed signal for ms. With a style (fx non-empty), plays that draft instead of
  // the saved one, so the editor can try a change before it is saved.
  bool web_preview(const std::string &key, uint32_t ms, const std::string &fx, const std::string &cm,
                   const std::string &colors, int sp, int br, int dir, int p);
  void web_preview_stop();
  std::string web_json();

 protected:
  struct Blob {
    uint8_t version;
    uint8_t style;
    uint8_t base;
    Style custom[M_COUNT];
  };

  Inputs inputs_();
  const Style &style_for_(uint8_t m) const;
  static bool patch_style_(Style &s, uint8_t m, const std::string &fx, const std::string &cm, const std::string &colors,
                           int sp, int br, int dir, int p);
  void schedule_save_();
  void save_();

  light::LightState *light_{nullptr};
  std::function<float()> timer_ratio_;
  std::function<float()> volume_;
  std::function<bool()> mic_muted_;
  std::function<bool()> speaker_silent_;
  RingStyleSelect *style_select_{nullptr};
  text_sensor::TextSensor *moment_sensor_{nullptr};
  CallbackManager<void()> preview_callback_;
  ESPPreferenceObject pref_;

  // Guarded by lock_: everything the web routes read or change.
  Mutex lock_;
  Style custom_[M_COUNT];
  uint8_t style_{P_CLASSIC};
  uint8_t base_{P_CLASSIC};
  std::string moment_{"idle"};
  std::string pv_key_;
  uint32_t pv_ms_{0};
  bool pv_has_style_{false};
  Style pv_style_{};
  bool pv_start_{false};
  bool pv_stop_{false};
  bool save_pending_{false};
  bool publish_style_{false};

  // Main loop only.
  int8_t moment_index_{-1};
  uint32_t t0_{0};
  int head_{0};
  int head0_{0};
  bool previewing_{false};
  uint32_t pv_until_{0};
  float pv_bright_{-1.0f};
  uint32_t save_at_{0};
  // The data the last Styled frame drew, for GET /api/sat1/ring's live ring. Written by the main
  // loop and read by httpd without the lock: aligned words and bytes the ESP32 cannot tear.
  float drawn_ratio_{0.0f};
  bool drawn_mic_{false};
  bool drawn_spk_{false};
};

class PreviewTrigger : public Trigger<> {
 public:
  explicit PreviewTrigger(RingFx *parent) {
    parent->add_on_preview_callback([this]() { this->trigger(); });
  }
};

}  // namespace esphome::satellite1_ring
