#include "satellite1_web_ui.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG = "satellite1_web_ui";

void Satellite1WebUI::setup() {
  this->handler_.set_loop_time_source(&this->max_loop_ms_);

  // The gate was registered from generated code long before this, but its token needs the
  // credentials the on_boot block installs at priority 600 - this setup runs at 250, after them
  // and before the listener starts at 249, so no request is ever checked against a half-built
  // token.
  this->gate_.setup();
  this->handler_.set_session_key_fn([this]() { return this->gate_.session_key(); });

  // Before the handler is registered, so a request arriving immediately cannot read an empty
  // selection and report that nothing is configured.
  this->selection_.setup();
  this->handler_.set_selection(&this->selection_);

  // Only a flag is set here. The callback runs on the httpd task, and what it ultimately starts are
  // Home Assistant actions, so the trigger has to be fired from the main loop instead.
  this->selection_.add_on_change_callback([this]() { this->selection_changed_.store(true); });

  // add_handler, not add_handler_without_auth: the app reaches every entity this device has,
  // Factory Reset and XMOS Erase Chip included.
  web_server_base::global_web_server_base->add_handler(&this->handler_);

#ifdef USE_SWITCH
  // The held-mute endpoint exists exactly when a mute switch is wired. Told to the handler here,
  // before the listener accepts anything, so the 404-vs-queued answer is fixed for the whole run.
  this->handler_.set_mute_hold_available(this->mute_switch_ != nullptr);
#endif

#ifdef USE_SAT1_WEB_UI_SENDSPIN
  if (this->sendspin_hub_ != nullptr) {
    // Every callback below fires on the main loop (the hub's own thread-context comments say so),
    // so the translation costs no locking here; the handler's setters own the httpd-side safety.
    //
    // Metadata replaces the whole state per message, absent field meaning cleared - and the hub
    // fans out a lost connection as an all-empty object, so one path covers updates and the clear.
    this->sendspin_hub_->add_metadata_update_callback([this](const sendspin::ServerMetadataStateObject &md) {
      this->handler_.media_set_meta(
          md.title.has_value() ? md.title->c_str() : nullptr, md.artist.has_value() ? md.artist->c_str() : nullptr,
          md.album.has_value() ? md.album->c_str() : nullptr,
          md.artwork_url.has_value() ? md.artwork_url->c_str() : nullptr,
          md.progress.has_value() ? md.progress->track_duration : 0);
    });
    this->sendspin_hub_->add_controller_state_callback([this](const sendspin::ServerStateControllerObject &st) {
      // The supported-command list becomes a bitmask, bit n = sendspin command enumerator n, so the
      // footer can offer exactly what the server offers - skip buttons on a queue, seek on a
      // seekable stream - without a second vocabulary on either end.
      uint16_t sup = 0;
      for (const auto cmd : st.supported_commands) {
        const auto bit = static_cast<unsigned>(cmd);
        if (bit < 16)
          sup |= static_cast<uint16_t>(1u << bit);
      }
      this->handler_.media_set_ctrl(st.shuffle, static_cast<uint8_t>(st.repeat), sup, st.seek_max_ms.value_or(0));
    });
    this->sendspin_hub_->add_controller_state_clear_callback([this]() { this->handler_.media_clear_ctrl(); });
  }
#endif

  this->last_loop_ms_ = millis();
}

void Satellite1WebUI::loop() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->last_loop_ms_;
  this->last_loop_ms_ = now;

  // Monotonic max, so a slow loop is never lost to a concurrent read that happened to land first.
  uint32_t seen = this->max_loop_ms_.load(std::memory_order_relaxed);
  while (elapsed > seen && !this->max_loop_ms_.compare_exchange_weak(seen, elapsed, std::memory_order_relaxed)) {
  }

  // The pairing window's lifecycle: expiry, the offline quiet-period judgement, and the open/close
  // events that drive the announcement and the LED. Triggers fire here rather than from the
  // endpoints because the endpoints run on the httpd task and what these start are scripts.
  this->gate_.tick(now);
  {
    std::string a, b;
    if (this->gate_.take_open_event(a, b))
      this->login_window_trigger_.trigger(a, b);
    if (this->gate_.take_close_event(a))
      this->login_window_end_trigger_.trigger(a);
  }

  // Collapses any number of refresh requests since the last iteration into one sync, which is what we
  // want: several tabs opening at once should ask Home Assistant a single time.
  if (this->handler_.take_ha_refresh_request())
    this->ha_refresh_trigger_.trigger();

  // Same reasoning, and the same collapsing: dragging through a tree writes several times in a
  // second, and each one would otherwise re-run every routing check.
  if (this->selection_changed_.exchange(false))
    this->selection_change_trigger_.trigger();

  // One per iteration rather than draining the queue, so the automation cannot be re-entered. Choosing
  // an assistant for a wake word can be up to four writes, which then land over four iterations.
  SelectWrite pending;
  if (this->handler_.take_select_write(pending))
    this->ha_select_trigger_.trigger(pending.entity, pending.option);

#ifdef USE_SWITCH
  // The tune-time held mute: a peer's tuner is measuring its wake word a room away and asked this
  // device not to answer it. The whole lifecycle lives here on the main loop; the endpoint only
  // queued. Ordered: request first (a release must not be aged out by the expiry check below it),
  // then the human override, then the TTL.
  if (this->mute_switch_ != nullptr) {
    const uint64_t now64 = millis_64();
    const int8_t req = this->handler_.take_mute_hold_request();
    if (req == 1) {
      if (!this->mute_hold_) {
        // First hold: capture what the person had chosen, then mute. turn_on() may be refused by
        // the template switch while the hardware slider is engaged - which is fine, the device is
        // already deaf - and mute_hold_seen_on_ keeps that unhonored request from reading as a
        // manual unmute below.
        this->mute_hold_ = true;
        this->mute_hold_prior_on_ = this->mute_switch_->state;
        this->mute_hold_seen_on_ = false;
        if (!this->mute_switch_->state)
          this->mute_switch_->turn_on();
        ESP_LOGI(TAG, "Tune-time mute held (was %s)", this->mute_hold_prior_on_ ? "muted" : "unmuted");
      }
      // Hold and keepalive are one request: either way the deadline moves. 60s against the
      // tuner's 20s cadence is three missed keepalives before a vanished browser self-heals.
      this->mute_hold_until_ = now64 + 60000;
    }
    if (this->mute_hold_) {
      if (this->mute_switch_->state)
        this->mute_hold_seen_on_ = true;
      if (req == 0 || (this->mute_hold_seen_on_ && !this->mute_switch_->state) || now64 > this->mute_hold_until_) {
        this->mute_hold_ = false;
        if (this->mute_hold_seen_on_ && !this->mute_switch_->state) {
          // A person unmuted at the device (or through Home Assistant) mid-hold: their hand
          // outranks the session (owner decision, September 23 2026). No restore, no re-mute.
          ESP_LOGI(TAG, "Tune-time mute overridden locally; hold cancelled");
        } else {
          // Release or expiry: put the switch back the way the hold found it. Restore only the
          // unmuted case - a device that was muted before the hold stays muted, and the switch is
          // already on.
          if (!this->mute_hold_prior_on_ && this->mute_switch_->state)
            this->mute_switch_->turn_off();
          ESP_LOGI(TAG, "Tune-time mute %s; %s", req == 0 ? "released" : "expired",
                   this->mute_hold_prior_on_ ? "staying muted (was muted before)" : "restored to unmuted");
        }
      }
    }
  }
#endif

  // The Music Assistant refresh, collapsed like the HA one and floored at two seconds besides:
  // browsers poll this on their own cadence while the footer's expanded view is open, several tabs
  // can, and each sync is an action call - one every couple of seconds is all the data can move.
  if (this->handler_.take_ma_refresh_request()) {
    if (now - this->ma_refresh_at_ >= 2000 || this->ma_refresh_at_ == 0) {
      this->ma_refresh_at_ = now;
      this->ma_refresh_trigger_.trigger();
    }
  }

  // One relayed Music Assistant command per iteration, for the select queue's reason. The volume and
  // seek values ride the queue as the strings the endpoint validated; they become numbers here, where
  // failure is impossible by construction rather than merely unhandled.
  MaWrite ma;
  if (this->handler_.take_ma_write(ma)) {
    switch (static_cast<MaCmd>(ma.kind)) {
      case MaCmd::LIKE:
        this->ma_like_trigger_.trigger(ma.entity);
        break;
      case MaCmd::JOIN:
        this->ma_join_trigger_.trigger(ma.entity, ma.arg);
        break;
      case MaCmd::UNJOIN:
        this->ma_unjoin_trigger_.trigger(ma.entity);
        break;
      case MaCmd::VOL:
        // 0-100 from the endpoint to media_player.volume_set's 0.0-1.0.
        this->ma_volume_trigger_.trigger(ma.entity, static_cast<float>(strtoul(ma.arg.c_str(), nullptr, 10)) / 100.0f);
        break;
      case MaCmd::SEEK:
        this->ma_seek_trigger_.trigger(ma.entity, static_cast<float>(strtoul(ma.arg.c_str(), nullptr, 10)));
        break;
      case MaCmd::NONE:
        break;
    }
  }

#ifdef USE_MICRO_WAKE_WORD
  // Not a trigger, because nothing in YAML has to happen - this is the whole of the work. It runs here
  // rather than in the endpoint because enabling a model writes NVS and races the inference task.
  this->handler_.apply_wake_word_requests();
#endif

#ifdef USE_MEDIA_PLAYER
  // Same shape as the wake words: the endpoint recorded the request, and make_call().perform()
  // belongs on the main loop, where every other caller of these players already lives.
  this->handler_.apply_media_requests();
#endif

#ifdef USE_SAT1_WEB_UI_SENDSPIN
  // Refreshed here rather than read in the endpoint, so the httpd task never calls into the hub -
  // the interpolation reads state the hub writes on this loop. One atomic store per iteration.
  if (this->sendspin_hub_ != nullptr)
    this->handler_.media_set_pos(this->sendspin_hub_->get_track_progress_ms());
#endif
}

char *Satellite1WebUI::stage_ha_payload(size_t capacity) { return this->handler_.stage_ha_payload(capacity); }

void Satellite1WebUI::commit_ha_payload(size_t len, int rung) {
  this->adopt_ha_area_(this->handler_.commit_ha_payload(len, rung));
}

void Satellite1WebUI::commit_ha_pages(int rung) { this->adopt_ha_area_(this->handler_.commit_ha_pages(rung)); }

void Satellite1WebUI::adopt_ha_area_(const char *json) {
  if (json == nullptr)
    return;

  // A find over the buffer rather than a JSON parse, and rather than a std::string built from it:
  // ArduinoJson is only linked in when something in the config uses capture_response, the area id is one
  // flat string in a payload we generate ourselves, and a copy of the payload to search it would put
  // several kilobytes on the internal heap - which is the whole thing the staging pair avoids. An empty
  // or missing `aid` is left alone rather than written, so a sync that happens while the device is
  // between areas cannot clear an area the switches are currently projecting.
  static const char *const KEY = "\"aid\":\"";
  const char *at = strstr(json, KEY);
  if (at == nullptr)
    return;
  const char *start = at + strlen(KEY);
  const char *end = strchr(start, '"');
  if (end == nullptr || end == start)
    return;
  this->selection_.set_own_area(std::string(start, static_cast<size_t>(end - start)));
}

void Satellite1WebUI::set_ha_payload(const std::string &json, int rung) {
  char *staged = this->stage_ha_payload(json.size() + 1);
  if (staged == nullptr)
    return;
  memcpy(staged, json.c_str(), json.size());
  this->commit_ha_payload(json.size(), rung);
}

void Satellite1WebUI::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 Web UI:\n"
                     "  Serving / and /ui/ from PROGMEM");
}

}  // namespace satellite1_web_ui
}  // namespace esphome
