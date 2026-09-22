#include "mww_runtime_loader.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// xTaskCreateWithCaps / vTaskDeleteWithCaps, for the PSRAM-stacked download task.
#include <freertos/idf_additions.h>

#include "esphome/components/json/json_util.h"
#include "esphome/components/logger/logger.h"
#include "esphome/components/network/util.h"
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

namespace esphome {
namespace mww_runtime_loader {

static const char *const TAG = "mww_runtime_loader";

/// The download task's stack. TLS buffers live on the heap, not here; this covers esp_http_client,
/// the JSON parse of a sub-kilobyte manifest, and our own frames. Sized like the other short-lived
/// network tasks in this firmware rather than measured to the byte.
///
/// Allocated in PSRAM (see start_job_), because 10KB of internal DRAM was this component's largest
/// transient cost, spent at boot - exactly when every URL slot re-downloads and internal RAM is
/// tightest. That placement is legal because this task never touches flash: it only does HTTP, TLS
/// and a JSON parse, and every NVS write and micro_wake_word mutation belongs to finalize_job_ on
/// the main loop. A task whose stack is in PSRAM must never perform a flash operation itself -
/// keep it that way.
static constexpr uint32_t WL_TASK_STACK = 10240;

/// Boot re-download backoff, capped: a device that came up before the router did should try again
/// soon, and one whose model URL has genuinely gone away should not hammer it forever.
static const uint32_t WL_RETRY_MS[] = {10 * 1000, 30 * 1000, 120 * 1000, 600 * 1000};

/// How long a read may go without a single byte before the fetch is declared stalled. read()
/// returning zero on a sized body gets waited out for transient hiccups - but bounded, because a
/// server that accepted the connection and then went quiet would otherwise park the download task,
/// and the single job slot with it, until reboot.
static constexpr uint32_t WL_STALL_MS = 30 * 1000;

#ifndef VERSION_CODE
#define VERSION_CODE(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif

static bool is_url_(const char *s) { return strncmp(s, "http://", 7) == 0 || strncmp(s, "https://", 8) == 0; }
static bool is_url_(const std::string &s) { return is_url_(s.c_str()); }

std::string MwwRuntimeLoader::runtime_id_(uint8_t slot) {
  // One stable id per slot rather than one per model. The id keys the model's enabled-state
  // preference, and micro_wake_word documents that every constructed model claims a preference
  // backend it never gives back - so ids must not multiply with every URL the customer tries.
  return slot == 0 ? "wl_slot_a" : "wl_slot_b";
}

void MwwRuntimeLoader::setup() {
#ifdef USE_LOG_LISTENERS
  // The tuner's score channel: micro_wake_word logs every detection's probabilities from its
  // loop(), and the logger hands registered listeners each line. Registered once here; the
  // callback is a no-op outside a tune session beyond one atomic load and, for mww lines, one
  // strcmp.
  if (logger::global_logger != nullptr)
    logger::global_logger->add_log_callback(this, MwwRuntimeLoader::log_callback_);
#endif

  // Compiled models are constructed and registered from generated code, which all runs before any
  // setup() - so this list is complete and, from here on, immutable. Copied so the httpd task can
  // read names without ever touching micro_wake_word.
  for (auto *model : this->mww_->get_wake_words()) {
    this->builtins_.push_back(BuiltinInfo{model->get_id(), model->get_wake_word()});
  }

  bool have_pref = false;
  for (uint8_t i = 0; i < WL_SLOTS; i++) {
    this->prefs_[i] = global_preferences->make_preference<SlotPrefData>(fnv1_hash(runtime_id_(i) + "_pref"));
    SlotPrefData d{};
    if (this->prefs_[i].load(&d) && d.used) {
      have_pref = true;
      d.spec[WL_SPEC_MAX - 1] = '\0';
      this->slots_[i].spec = d.spec;
      // Below WL_TUNED_MIN only 0 means anything; 1..4 are step numbers a dev build persisted
      // before the tuner replaced the presets, and they read as the model default.
      this->slots_[i].cutoff = d.cutoff >= WL_TUNED_MIN ? d.cutoff : 0;
    }
  }

  // Two slots persisted the same spec (possible only through an old firmware's races): keep the
  // first, empty the second. Two slots holding one model breaks the enabled-iff-in-a-slot
  // invariant every unload and reconcile pass relies on.
  if (!this->slots_[0].spec.empty() && this->slots_[0].spec == this->slots_[1].spec) {
    ESP_LOGW(TAG, "Both slots persisted \"%s\"; emptying slot 1", this->slots_[1].spec.c_str());
    this->slots_[1].spec.clear();
    this->slots_[1].cutoff = 0;
    this->save_slot_(1);
  }

  if (!have_pref) {
    // First boot on this firmware: adopt whatever was enabled, in model order, so an OTA update
    // changes nothing the customer can hear. A factory-fresh device has only the first model
    // enabled (micro_wake_word's own default), which lands it in the primary slot.
    uint8_t at = 0;
    for (const auto &b : this->builtins_) {
      auto *model = this->builtin_by_id_(b.id);
      if (model != nullptr && model->is_enabled() && at < WL_SLOTS) {
        this->slots_[at++].spec = b.id;
      }
    }
    if (at == 0 && !this->builtins_.empty()) {
      this->slots_[0].spec = this->builtins_[0].id;
    }
    for (uint8_t i = 0; i < WL_SLOTS; i++)
      this->save_slot_(i);
  }

  // Enforce the slot picture over the compiled models: enabled iff a slot names it. A URL slot has
  // nothing to enable yet - it goes to WAITING and loop() fetches it once the network is up.
  for (uint8_t i = 0; i < WL_SLOTS; i++) {
    Slot &s = this->slots_[i];
    if (s.spec.empty())
      continue;
    if (is_url_(s.spec)) {
      s.state = SLOT_WAITING;
    } else if (auto *model = this->builtin_by_id_(s.spec)) {
      model->enable();
      s.model_id = s.spec;
      s.word = model->get_wake_word();
    } else {
      // A spec naming a model this build no longer compiles in. Empty the slot rather than carry a
      // ghost.
      s.spec.clear();
      this->save_slot_(i);
    }
  }
  for (const auto &b : this->builtins_) {
    bool held = false;
    for (auto &s : this->slots_)
      held |= (s.model_id == b.id);
    if (!held) {
      if (auto *model = this->builtin_by_id_(b.id))
        model->disable();
    }
  }

  // Let the boot settle - and Home Assistant's on-connect configuration write land - before the
  // drift adoption runs. Without this, adoption could race the API connection's own replay.
  // millis_64() for every deadline in this component: plain millis() wraps at 49.7 days and a
  // direct comparison against a wrapped counter reads "not yet" for the next 49 - these devices
  // stay up for months.
  this->reconcile_after_ms_ = millis_64() + 15000;
  this->publish_view_();
}

void MwwRuntimeLoader::loop() {
  // A built-in word's persisted tuned threshold, re-asserted once after every setup() has run -
  // deliberately not in our own setup(), so it lands after anything an entity restore may have
  // written to the same cutoffs. Runtime slots need nothing here: theirs rides the download's
  // finalize.
  if (!this->overrides_applied_) {
    this->overrides_applied_ = true;
    for (auto &s : this->slots_) {
      if (s.cutoff == 0 || s.model_id.empty())
        continue;
      this->apply_configured_cutoff_(s);
    }
  }

  // An expired tune session restores the configured threshold - the browser is gone, and a device
  // left on the probe floor would false-trigger until someone noticed.
  if (this->tune_slot_.load(std::memory_order_relaxed) >= 0 && millis_64() > this->tune_deadline_ms_)
    this->end_tune_(true);

  if (this->job_.active.load(std::memory_order_acquire)) {
    if (this->job_.done.load(std::memory_order_acquire))
      this->finalize_job_();
    return;
  }

  // Requests from the web handler, newest write per slot wins. Applying one can start a download,
  // which occupies the single job - anything still pending is simply picked up next iteration.
  if (this->req_pending_.load(std::memory_order_relaxed)) {
    uint8_t slot = WL_SLOTS;
    std::string spec;
    int16_t cutoff = -1;
    uint8_t cutoff_slot = WL_SLOTS;
    int8_t tune = -1;
    {
      LockGuard guard{this->req_lock_};
      // The tune exchange sits under the same lock the writers take (queue_tune stores under it
      // too): exchanged outside, a request arriving between the exchange and the req_pending_
      // recomputation below was overwritten to "nothing pending" and sat unprocessed - a dropped
      // close left the model on the probe floor for the whole TTL.
      tune = this->tune_req_.exchange(-1, std::memory_order_relaxed);
      for (uint8_t i = 0; i < WL_SLOTS; i++) {
        if (this->cutoff_req_[i] >= 0) {
          cutoff = this->cutoff_req_[i];
          cutoff_slot = i;
          this->cutoff_req_[i] = -1;
          break;
        }
      }
      if (cutoff_slot == WL_SLOTS) {
        for (uint8_t i = 0; i < WL_SLOTS; i++) {
          if (this->slot_req_[i]) {
            slot = i;
            spec = this->slot_req_spec_[i];
            this->slot_req_[i] = false;
            break;
          }
        }
      }
      bool more = false;
      for (uint8_t i = 0; i < WL_SLOTS; i++)
        more |= this->slot_req_[i] || this->cutoff_req_[i] >= 0;
      this->req_pending_.store(more, std::memory_order_relaxed);
    }
    if (tune >= 0)
      this->apply_tune_(static_cast<uint8_t>(tune / 2), (tune & 1) != 0);
    if (cutoff_slot < WL_SLOTS) {
      this->apply_cutoff_(cutoff_slot, static_cast<uint8_t>(cutoff));
    } else if (slot < WL_SLOTS) {
      this->apply_request_(slot, spec);
    }
    return;
  }

  // Boot re-download: a slot that persisted a URL fetches it again, with backoff, once there is a
  // network to fetch over. One at a time, sharing the single job with everything else.
  const uint64_t now = millis_64();
  for (uint8_t i = 0; i < WL_SLOTS; i++) {
    Slot &s = this->slots_[i];
    const bool retryable = s.state == SLOT_WAITING || (s.state == SLOT_ERROR && s.model_id.empty() && is_url_(s.spec));
    if (retryable && now >= s.next_retry_ms) {
      if (!network::is_connected()) {
        if (s.state != SLOT_WAITING) {
          s.state = SLOT_WAITING;
          this->publish_view_();
        }
        s.next_retry_ms = now + 2000;
        continue;
      }
      this->start_job_(i, true);
      return;
    }
  }

  if (now >= this->reconcile_after_ms_)
    this->reconcile_ha_();
}

/* ---- httpd-task API ---- */

bool MwwRuntimeLoader::queue_slot(uint8_t i, const std::string &spec) {
  if (i >= WL_SLOTS)
    return false;
  std::string s = spec == "none" ? std::string() : spec;
  if (s.size() >= WL_SPEC_MAX)
    return false;
  if (!s.empty() && !is_url_(s) && !this->is_builtin_(s))
    return false;
  // No characters that could break the URL out of a log line or a JSON string. A manifest URL is
  // ASCII by construction; anything else here is not a URL the loader should chase.
  for (unsigned char c : s) {
    if (c <= ' ' || c == '"' || c == '\\' || c > 0x7e)
      return false;
  }
  {
    LockGuard guard{this->view_lock_};
    // The other slot already holds (or is fetching) this word - a duplicate would be two models
    // listening for the same phrase, and add_runtime_model would refuse the second anyway.
    if (!s.empty() && strcmp(this->view_[1 - i].spec, s.c_str()) == 0)
      return false;
  }
  {
    LockGuard guard{this->req_lock_};
    this->slot_req_[i] = true;
    // Bounded above (s.size() < WL_SPEC_MAX) and NUL-free by the character walk, so the copy with
    // its terminator always fits the fixed array.
    memcpy(this->slot_req_spec_[i], s.c_str(), s.size() + 1);
  }
  this->req_pending_.store(true, std::memory_order_relaxed);
  return true;
}

bool MwwRuntimeLoader::queue_cutoff(uint8_t i, uint8_t value) {
  if (i >= WL_SLOTS || (value != 0 && (value < WL_TUNED_MIN || value > 254)))
    return false;
  {
    LockGuard guard{this->req_lock_};
    this->cutoff_req_[i] = value;
  }
  this->req_pending_.store(true, std::memory_order_relaxed);
  return true;
}

bool MwwRuntimeLoader::queue_tune(uint8_t i, bool on) {
  if (i >= WL_SLOTS)
    return false;
  {
    // Under req_lock_ so the drain in loop() - which exchanges this under the same lock - can
    // never lose a request written between its exchange and its req_pending_ recomputation.
    LockGuard guard{this->req_lock_};
    this->tune_req_.store(static_cast<int8_t>(i * 2 + (on ? 1 : 0)), std::memory_order_relaxed);
  }
  this->req_pending_.store(true, std::memory_order_relaxed);
  return true;
}

void MwwRuntimeLoader::snapshot(SlotView out[WL_SLOTS], uint32_t &dl, uint32_t &total, int &dl_slot) {
  {
    LockGuard guard{this->view_lock_};
    // Materialized field by field: the stored view is fixed char arrays (see SlotViewStore), the
    // endpoint's copy is std::string - transient for the request, where the store is forever.
    for (uint8_t i = 0; i < WL_SLOTS; i++) {
      const SlotViewStore &v = this->view_[i];
      out[i].spec.assign(v.spec);
      out[i].word.assign(v.word);
      out[i].id.assign(v.id);
      out[i].state = v.state;
      out[i].error = v.error;
      out[i].cutoff = v.cutoff;
      out[i].runtime = v.runtime;
    }
  }
  if (this->job_.active.load(std::memory_order_acquire)) {
    dl = this->job_.bytes.load(std::memory_order_relaxed);
    total = this->job_.total.load(std::memory_order_relaxed);
    dl_slot = this->job_.slot;
  } else {
    dl = total = 0;
    dl_slot = -1;
  }
}

/* ---- main-loop application ---- */

micro_wake_word::WakeWordModel *MwwRuntimeLoader::builtin_by_id_(const std::string &id) {
  for (auto *model : this->mww_->get_wake_words()) {
    if (model->get_id() == id && model->get_id() != runtime_id_(0) && model->get_id() != runtime_id_(1))
      return model;
  }
  return nullptr;
}

bool MwwRuntimeLoader::is_builtin_(const std::string &spec) {
  for (const auto &b : this->builtins_) {
    if (b.id == spec)
      return true;
  }
  return false;
}

void MwwRuntimeLoader::unload_slot_(Slot &s) {
  if (s.model_id.empty())
    return;
  if (s.model_id == runtime_id_(0) || s.model_id == runtime_id_(1)) {
    this->mww_->remove_runtime_model(s.model_id);
  } else if (auto *model = this->builtin_by_id_(s.model_id)) {
    model->disable();
  }
  s.model_id.clear();
  s.word.clear();
}

void MwwRuntimeLoader::apply_request_(uint8_t i, const std::string &spec) {
  Slot &s = this->slots_[i];

  // Changing what the slot holds ends any tune session on it - the probe floor belongs to the
  // word it was measuring, never to its successor.
  if (this->tune_slot_.load(std::memory_order_relaxed) == static_cast<int8_t>(i))
    this->end_tune_(false);

  if (spec == s.spec && s.state == SLOT_READY) {
    this->publish_view_();
    return;
  }

  // The duplicate guard again, here on the main loop where it cannot race: queue_slot checks the
  // published view, but two requests queued back to back both pass that check before either
  // applies - and two slots holding one model breaks the enabled-iff-in-a-slot invariant (emptying
  // one slot would disable the model the other still claims, and reconciliation would then empty
  // that one too). Refused with the error the app already knows how to say.
  if (!spec.empty()) {
    const Slot &other = this->slots_[1 - i];
    const std::string &held = !other.pending.empty() ? other.pending : other.spec;
    if (held == spec) {
      s.pending.clear();
      s.state = SLOT_ERROR;
      s.error = ERR_REFUSED;
      this->publish_view_();
      return;
    }
  }

  if (spec.empty()) {
    const bool removed_runtime = s.model_id == runtime_id_(i);
    this->unload_slot_(s);
    s.spec.clear();
    s.pending.clear();
    s.state = SLOT_READY;
    s.error = ERR_NONE;
    s.cutoff = 0;
    this->save_slot_(i);
    this->reconcile_after_ms_ = millis_64() + 3000;
    if (removed_runtime)
      this->nudge_ha_();
    this->publish_view_();
    return;
  }

  if (this->is_builtin_(spec)) {
    auto *model = this->builtin_by_id_(spec);
    if (model == nullptr)
      return;
    const bool removed_runtime = s.model_id == runtime_id_(i);
    this->unload_slot_(s);
    model->enable();
    s.spec = spec;
    s.pending.clear();
    s.model_id = spec;
    s.word = model->get_wake_word();
    s.state = SLOT_READY;
    s.error = ERR_NONE;
    s.cutoff = 0;
    this->save_slot_(i);
    this->reconcile_after_ms_ = millis_64() + 3000;
    if (removed_runtime)
      this->nudge_ha_();
    this->publish_view_();
    return;
  }

  // A URL: the previous word keeps listening until the replacement has downloaded and loaded, so a
  // bad link can never leave the device deaf. The new spec is persisted only on success for the
  // same reason - a reboot mid-failure must come back with the word that was working.
  s.pending = spec;
  s.error = ERR_NONE;
  this->start_job_(i, false);
}

void MwwRuntimeLoader::apply_cutoff_(uint8_t i, uint8_t value) {
  Slot &s = this->slots_[i];
  if (s.model_id.empty())
    return;
  auto *model = this->mww_->get_model_by_id(s.model_id);
  if (model == nullptr)
    return;
  s.cutoff = value;
  // Deliberately also while this slot's tune session is live: applying the recommendation is the
  // session's confirm step, and the floor has to yield to the real threshold for it.
  model->set_probability_cutoff(value != 0 ? value : model->get_default_probability_cutoff());
  ESP_LOGI(TAG, "Sensitivity for \"%s\": cutoff %u%s", s.word.c_str(), model->get_probability_cutoff(),
           value == 0 ? " (model default)" : " (tuned)");
  this->save_slot_(i);
  this->publish_view_();
}

void MwwRuntimeLoader::nudge_ha_() {
#ifdef USE_API
  // Home Assistant caches the wake word list it fetched when its API connection came up, and this
  // generation of the protocol has no configuration-changed push - so a word downloaded after that
  // is invisible to the ESPHome device page (the "Hey Nexus" the customer just picked shows as
  // nothing) until Home Assistant asks again. The one lever that exists is the connection itself:
  // drop it cleanly and Home Assistant reconnects within seconds and re-fetches, and the fresh
  // list carries the new word's real phrase plus the active set its selects read. Delayed a beat
  // so the swap's HTTP response and the app's follow-up writes drain first.
  this->set_timeout("wl_nudge_ha", 2500, []() {
    if (api::global_api_server == nullptr)
      return;
    bool any = false;
    for (const auto &conn : api::global_api_server->active_clients()) {
      conn->on_fatal_error();
      any = true;
    }
    if (any)
      ESP_LOGI(TAG, "Wake word list changed; dropping the API connection so Home Assistant re-reads it");
  });
#endif
}

void MwwRuntimeLoader::apply_configured_cutoff_(Slot &s) {
  if (s.model_id.empty())
    return;
  auto *model = this->mww_->get_model_by_id(s.model_id);
  if (model == nullptr)
    return;
  model->set_probability_cutoff(s.cutoff != 0 ? s.cutoff : model->get_default_probability_cutoff());
}

void MwwRuntimeLoader::apply_tune_(uint8_t i, bool on) {
  const int8_t active = this->tune_slot_.load(std::memory_order_relaxed);
  if (!on) {
    if (active == static_cast<int8_t>(i))
      this->end_tune_(false);
    return;
  }
  if (active == static_cast<int8_t>(i)) {
    // Keepalive: the panel is still open.
    this->tune_deadline_ms_ = millis_64() + WL_TUNE_TTL_MS;
    return;
  }
  if (active >= 0)
    this->end_tune_(false);

  Slot &s = this->slots_[i];
  // A loaded model is what a session needs - not a READY state. A slot whose last swap failed
  // sits in ERROR while its previous word keeps listening, and that word is as tunable as ever;
  // requiring READY made one bad URL attempt lock the tuner out until a reboot.
  if (s.model_id.empty() || s.state == SLOT_DOWNLOADING)
    return;
  auto *model = this->mww_->get_model_by_id(s.model_id);
  if (model == nullptr)
    return;

  {
    LockGuard guard{this->tune_lock_};
    this->tune_word_ = s.word;
    this->tune_events_.clear();
  }
  this->tune_deadline_ms_ = millis_64() + WL_TUNE_TTL_MS;
  this->tune_slot_.store(static_cast<int8_t>(i), std::memory_order_release);
  model->set_probability_cutoff(WL_TUNE_FLOOR);
  ESP_LOGI(TAG, "Tune session opened for \"%s\" (probe floor %u)", s.word.c_str(), WL_TUNE_FLOOR);
}

void MwwRuntimeLoader::end_tune_(bool expired) {
  const int8_t active = this->tune_slot_.exchange(-1, std::memory_order_acq_rel);
  if (active < 0)
    return;
  this->apply_configured_cutoff_(this->slots_[active]);
  {
    // The ring and the word are session state, and the session is over - hand the heap back rather
    // than holding two dozen events until the next open clears them. Safe against the log callback:
    // it tests tune_slot_ (already -1) before ever taking this lock, and the endpoint serves the
    // tune block only while a session is live.
    LockGuard guard{this->tune_lock_};
    std::vector<TuneEvent>().swap(this->tune_events_);
    std::string().swap(this->tune_word_);
  }
  ESP_LOGI(TAG, "Tune session for slot %d closed%s", active, expired ? " (expired)" : "");
}

void MwwRuntimeLoader::tune_snapshot(TuneView &out) {
  out.slot = this->tune_slot_.load(std::memory_order_acquire);
  LockGuard guard{this->tune_lock_};
  out.seq = this->tune_seq_;
  out.events = this->tune_events_;
}

#ifdef USE_LOG_LISTENERS
void MwwRuntimeLoader::log_callback_(void *self, uint8_t level, const char *tag, const char *message, size_t len) {
  auto *loader = static_cast<MwwRuntimeLoader *>(self);
  if (loader->tune_slot_.load(std::memory_order_relaxed) < 0)
    return;
  if (strcmp(tag, "micro_wake_word") != 0)
    return;
  loader->on_mww_log_(message);
}

void MwwRuntimeLoader::on_mww_log_(const char *message) {
  // The two lines this parses are pinned by requirements.txt's exact ESPHome version - they live at
  // micro_wake_word.cpp:488-495 in 2026.8.1. An ESPHome bump must re-check both format strings; if
  // they drift, the tuner records nothing and the panel reports scores unavailable rather than
  // guessing. Both lines are logged from the component's loop(), so this runs on the main loop for
  // every message it can match.
  //
  // `message` is the logger's whole formatted buffer - ANSI colour runs, the "[D][tag:line]:"
  // header, then the text (notify_listeners_ hands callbacks buf.data, the same bytes the console
  // gets). So the payload is *found*, not assumed to start the string; sscanf's %f then stops
  // cleanly at the trailing colour-reset escape. Hardware-found: the first build parsed from
  // offset zero and recorded nothing while the log visibly scored every attempt.
  char word[64];
  float avg = 0.0f, peak = 0.0f;
  bool vad = false;
  const char *at = strstr(message, "Detected '");
  if (at != nullptr &&
      sscanf(at, "Detected '%63[^']' with sliding average probability is %f and max probability is %f", word, &avg,
             &peak) == 3) {
    // fall through to record
  } else if ((at = strstr(message, "Wake word model predicts '")) != nullptr &&
             sscanf(at, "Wake word model predicts '%63[^']'", word) == 1) {
    vad = true;
  } else {
    return;
  }

  LockGuard guard{this->tune_lock_};
  if (this->tune_word_ != word)
    return;
  TuneEvent ev;
  ev.peak = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, peak)) * 255.0f);
  ev.avg = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, avg)) * 255.0f);
  ev.vad_blocked = vad;
  ev.at_ms = millis();
  this->tune_events_.push_back(ev);
  if (this->tune_events_.size() > WL_TUNE_RING)
    this->tune_events_.erase(this->tune_events_.begin());
  this->tune_seq_++;
}
#endif  // USE_LOG_LISTENERS

void MwwRuntimeLoader::start_job_(uint8_t slot, bool boot) {
  Slot &s = this->slots_[slot];
  Job &j = this->job_;
  j.slot = slot;
  j.boot = boot;
  j.manifest_url = boot ? s.spec : s.pending;
  j.error = ERR_NONE;
  j.word.clear();
  j.langs.clear();
  j.data.reset();
  j.bytes.store(0, std::memory_order_relaxed);
  j.total.store(0, std::memory_order_relaxed);
  j.done.store(false, std::memory_order_relaxed);
  j.active.store(true, std::memory_order_release);

  s.state = SLOT_DOWNLOADING;
  this->publish_view_();

  // WithCaps puts the stack in PSRAM and keeps the TCB internal on its own (FreeRTOS requires
  // that); the internal-caps retry covers a PSRAM-less board through the same call so job_task's
  // vTaskDeleteWithCaps matches however the task was created. See WL_TASK_STACK for why PSRAM is
  // safe here and what would make it stop being safe.
  if (xTaskCreateWithCaps(MwwRuntimeLoader::job_task, "wl_download", WL_TASK_STACK, this, 2, nullptr,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS &&
      xTaskCreateWithCaps(MwwRuntimeLoader::job_task, "wl_download", WL_TASK_STACK, this, 2, nullptr,
                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) != pdPASS) {
    j.error = ERR_NO_MEMORY;
    j.done.store(true, std::memory_order_release);
  }
}

void MwwRuntimeLoader::job_task(void *param) {
  auto *self = static_cast<MwwRuntimeLoader *>(param);
  self->run_job_();
  self->job_.done.store(true, std::memory_order_release);
  // The WithCaps pair: a plain vTaskDelete would leak the stack and TCB buffers WithCaps allocated
  // (they are static allocations as far as FreeRTOS is concerned). Self-delete is supported - IDF
  // spawns a momentary cleanup task to free the buffers.
  vTaskDeleteWithCaps(nullptr);
}

uint8_t MwwRuntimeLoader::fetch_(const std::string &url, PsramString &out, size_t cap) {
  auto container = this->http_->get(url);
  if (container == nullptr)
    return ERR_FETCH;
  if (container->status_code != 200) {
    container->end();
    return ERR_FETCH;
  }
  const int64_t len = container->content_length;
  if (len > 0 && static_cast<size_t>(len) > cap) {
    container->end();
    return ERR_TOO_BIG;
  }
  out.clear();
  out.reserve(len > 0 ? static_cast<size_t>(len) : 1024);
  uint8_t buf[1024];
  uint32_t last_progress = millis();
  while (out.size() < cap) {
    int n = container->read(buf, sizeof(buf));
    if (n < 0) {
      container->end();
      return ERR_FETCH;
    }
    if (n == 0) {
      // EOF for an unsized body. For a sized one mid-body it is a hiccup - waited out, but only
      // to WL_STALL_MS (unsigned subtraction, wrap-safe), so a quiet server cannot hold the task.
      if (len <= 0 || out.size() >= static_cast<size_t>(len) || millis() - last_progress > WL_STALL_MS)
        break;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    last_progress = millis();
    out.append(reinterpret_cast<const char *>(buf), static_cast<size_t>(n));
  }
  container->end();
  if (len > 0 && out.size() != static_cast<size_t>(len))
    return ERR_FETCH;
  return ERR_NONE;
}

void MwwRuntimeLoader::run_job_() {
  Job &j = this->job_;

  // 1. The manifest. Small JSON; capped hard, so a URL pointing at a .tflite (or a web page) fails
  //    here as "not a manifest" territory rather than filling memory. PSRAM, because the cap is
  //    8KB and the internal heap should never carry a body that size even transiently.
  PsramString manifest;
  uint8_t verdict = this->fetch_(j.manifest_url, manifest, WL_MANIFEST_MAX);
  if (verdict != ERR_NONE) {
    j.error = verdict == ERR_TOO_BIG ? ERR_NOT_MANIFEST : ERR_FETCH;
    return;
  }

  std::string model_rel;
  uint32_t step = 10;
  uint32_t min_ma = 0, min_mi = 0, min_pa = 0;
  int version = 0;
  float cutoff_f = 0.97f;
  bool has_micro = false;
  std::string type;

  // The pointer-and-length overload, because `manifest` is a PSRAM basic_string rather than the
  // std::string the convenience overload takes - converting would copy the body onto the internal
  // heap, which is the exact move the PSRAM placement exists to avoid.
  bool parsed = json::parse_json(reinterpret_cast<const uint8_t *>(manifest.c_str()), manifest.size(),
                                 [&](JsonObject root) -> bool {
    type = (const char *) (root["type"] | "");
    j.word = (const char *) (root["wake_word"] | "");
    version = root["version"] | 0;
    model_rel = (const char *) (root["model"] | "");
    if (root["trained_languages"].is<JsonArray>()) {
      for (JsonVariant v : root["trained_languages"].as<JsonArray>()) {
        const char *lang = v.as<const char *>();
        if (lang != nullptr)
          j.langs.emplace_back(lang);
      }
    }
    if (root["micro"].is<JsonObject>()) {
      has_micro = true;
      JsonObject micro = root["micro"];
      cutoff_f = micro["probability_cutoff"] | 0.97f;
      j.window = micro["sliding_window_size"] | 5;
      step = micro["feature_step_size"] | 10;
      j.arena = micro["tensor_arena_size"] | 30000;
      const char *minv = micro["minimum_esphome_version"] | "";
      // Accepts "2024.7" and "2024.7.0" alike; a manifest with garbage here reads as 0.0.0, which
      // every firmware satisfies - the tflite validation downstream is the gate that matters.
      sscanf(minv, "%" PRIu32 ".%" PRIu32 ".%" PRIu32, &min_ma, &min_mi, &min_pa);
    }
    return true;
  });

  if (!parsed || type != "micro" || !has_micro || j.word.empty() || model_rel.empty()) {
    j.error = ERR_NOT_MANIFEST;
    return;
  }

  // The phrase is display copy everywhere it goes - this card, Home Assistant's wake word
  // dropdown, the detections history - and the bulk-trained catalogs write it as a slug:
  // "hey_alice" where the list the customer picked from said "Hey Alice" (owner hit exactly that).
  // Underscores become spaces and each word's first letter capitalizes; a manifest that already
  // says "Hey Jarvis" passes through untouched.
  bool cap = true;
  for (auto &ch : j.word) {
    if (ch == '_')
      ch = ' ';
    if (cap && ch >= 'a' && ch <= 'z')
      ch -= 'a' - 'A';
    cap = ch == ' ';
  }
  if (version != 2) {
    j.error = ERR_VERSION;
    return;
  }
  if (step != this->mww_->get_features_step_size()) {
    j.error = ERR_STEP;
    return;
  }
  if (VERSION_CODE(min_ma, min_mi, min_pa) > ESPHOME_VERSION_CODE) {
    j.error = ERR_TOO_NEW;
    return;
  }
  if (j.arena == 0 || j.arena > WL_ARENA_MAX) {
    j.error = ERR_TOO_BIG;
    return;
  }
  if (cutoff_f <= 0.0f || cutoff_f > 1.0f)
    cutoff_f = 0.97f;
  j.cutoff = static_cast<uint8_t>(cutoff_f * 255.0f);
  if (j.window == 0 || j.window > 50)
    j.window = 5;

  // 2. The model URL, resolved against the manifest when relative - which is how every collection
  //    writes it ("model": "achim.tflite" next to achim.json).
  std::string model_url;
  if (is_url_(model_rel)) {
    model_url = model_rel;
  } else {
    if (model_rel.rfind("./", 0) == 0)
      model_rel.erase(0, 2);
    const size_t cut = j.manifest_url.rfind('/');
    if (cut == std::string::npos) {
      j.error = ERR_NOT_MANIFEST;
      return;
    }
    model_url = j.manifest_url.substr(0, cut + 1) + model_rel;
  }

  // 3. The model itself, streamed straight into the PSRAM buffer micro_wake_word will run it from.
  auto container = this->http_->get(model_url);
  if (container == nullptr) {
    j.error = ERR_DOWNLOAD;
    return;
  }
  if (container->status_code != 200) {
    container->end();
    j.error = ERR_DOWNLOAD;
    return;
  }
  const int64_t len = container->content_length;
  if (len <= 0) {
    // No length means no way to size the buffer or to know a truncated read from a finished one.
    // Every host these models live on sends Content-Length; one that does not is refused.
    container->end();
    j.error = ERR_DOWNLOAD;
    return;
  }
  if (static_cast<size_t>(len) > WL_MODEL_MAX) {
    container->end();
    j.error = ERR_TOO_BIG;
    return;
  }

  auto data = std::make_shared<micro_wake_word::ModelData>();
  if (!data->allocate(static_cast<size_t>(len))) {
    container->end();
    j.error = ERR_NO_MEMORY;
    return;
  }
  j.total.store(static_cast<uint32_t>(len), std::memory_order_relaxed);

  uint8_t *write = data->get_write_pointer();
  size_t off = 0;
  uint32_t last_progress = millis();
  while (off < static_cast<size_t>(len)) {
    int n = container->read(write + off, std::min<size_t>(4096, static_cast<size_t>(len) - off));
    if (n < 0)
      break;
    if (n == 0) {
      // read() returning zero mid-body is a stall, not EOF for a sized response; give the socket a
      // moment rather than spinning - but only WL_STALL_MS of moments (unsigned subtraction,
      // wrap-safe). Unbounded, a server that accepted the connection and then went quiet parked
      // this task forever, and with it the single job every swap and boot re-download shares.
      if (millis() - last_progress > WL_STALL_MS)
        break;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    last_progress = millis();
    off += static_cast<size_t>(n);
    j.bytes.store(static_cast<uint32_t>(off), std::memory_order_relaxed);
  }
  container->end();

  if (off != static_cast<size_t>(len)) {
    j.error = ERR_DOWNLOAD;
    return;
  }
  if (!data->validate_and_mark_ready()) {
    j.error = ERR_INVALID;
    return;
  }
  j.data = std::move(data);
}

void MwwRuntimeLoader::finalize_job_() {
  Job &j = this->job_;
  Slot &s = this->slots_[j.slot];
  j.active.store(false, std::memory_order_release);

  if (j.error != ERR_NONE || !j.data) {
    ESP_LOGW(TAG, "Wake word download for slot %u failed (error %u): %s", j.slot, j.error, j.manifest_url.c_str());
    j.data.reset();
    s.pending.clear();
    s.error = j.error != ERR_NONE ? j.error : ERR_DOWNLOAD;
    if (j.boot) {
      // The word is gone until a fetch succeeds, so keep trying - backing off to every ten
      // minutes, forever, because the URL may simply be ahead of the household's DNS.
      const uint8_t at = std::min<uint8_t>(s.retries, sizeof(WL_RETRY_MS) / sizeof(WL_RETRY_MS[0]) - 1);
      s.next_retry_ms = millis_64() + WL_RETRY_MS[at];
      s.retries++;
      s.state = SLOT_ERROR;
    } else {
      // The previous word never stopped listening; the slot just reports why the swap failed.
      s.state = SLOT_ERROR;
    }
    this->release_job_strings_();
    this->publish_view_();
    return;
  }

  const std::string id = runtime_id_(j.slot);
  const std::string old_model = s.model_id;

  // A re-download into the same slot (boot, or a retry) must clear the previous instance of its
  // own id before add_runtime_model sees a duplicate.
  if (old_model == id) {
    this->mww_->remove_runtime_model(id);
    s.model_id.clear();
  }

  auto model = make_unique<micro_wake_word::WakeWordModel>(id, j.data, j.cutoff, j.window, j.word, j.langs,
                                                           j.arena);
  if (!this->mww_->add_runtime_model(std::move(model))) {
    j.data.reset();
    s.pending.clear();
    s.state = SLOT_ERROR;
    s.error = ERR_REFUSED;
    this->release_job_strings_();
    this->publish_view_();
    return;
  }

  // The displaced word goes now, after the replacement is in - never before, so a failure above
  // leaves the old word running.
  if (!old_model.empty() && old_model != id) {
    Slot displaced;
    displaced.model_id = old_model;
    this->unload_slot_(displaced);
  }

  auto *loaded = this->mww_->get_model_by_id(id);
  if (loaded != nullptr) {
    loaded->enable();
  }

  // A different word starts at its model's own tuning; the same URL fetched again - a boot
  // re-download, or a retry - keeps the tuned threshold the customer measured and re-applies it
  // now that the model is loaded.
  const bool same_word = s.spec == j.manifest_url;
  s.spec = j.manifest_url;
  s.pending.clear();
  s.model_id = id;
  s.word = j.word;
  s.state = SLOT_READY;
  s.error = ERR_NONE;
  s.retries = 0;
  if (!same_word)
    s.cutoff = 0;
  if (s.cutoff != 0 && loaded != nullptr)
    loaded->set_probability_cutoff(s.cutoff);
  this->save_slot_(j.slot);
  j.data.reset();
  this->reconcile_after_ms_ = millis_64() + 3000;
  if (!j.boot)
    this->nudge_ha_();
  ESP_LOGI(TAG, "Wake word \"%s\" loaded into slot %u from %s", s.word.c_str(), j.slot, s.spec.c_str());
  this->release_job_strings_();
  this->publish_view_();
}

void MwwRuntimeLoader::release_job_strings_() {
  Job &j = this->job_;
  // swap-with-empty rather than clear(): clear() keeps the capacity, and the whole point here is
  // handing a URL-sized buffer back to the internal heap between downloads.
  std::string().swap(j.manifest_url);
  std::string().swap(j.word);
  std::vector<std::string>().swap(j.langs);
}

void MwwRuntimeLoader::reconcile_ha_() {
  // Never while a download is mutating a slot: adoption would read a half-applied picture.
  for (auto &s : this->slots_) {
    if (s.state == SLOT_DOWNLOADING)
      return;
  }

  // Home Assistant's wake word select writes land as raw enable/disable flips over every model
  // (voice_assistant::on_set_configuration). The slots adopt them: a slot whose model was switched
  // off empties, and a compiled model switched on takes an empty slot. The device stays the source
  // of truth - a word that cannot fit the two slots is switched back off.
  bool drift = false;
  auto in_slot = [&](const std::string &id) {
    for (auto &s : this->slots_) {
      if (s.model_id == id)
        return true;
    }
    return false;
  };
  for (auto *model : this->mww_->get_wake_words()) {
    if (model->is_enabled() != in_slot(model->get_id())) {
      drift = true;
      break;
    }
  }
  if (!drift)
    return;

  for (uint8_t i = 0; i < WL_SLOTS; i++) {
    Slot &s = this->slots_[i];
    if (s.model_id.empty())
      continue;
    auto *model = this->mww_->get_model_by_id(s.model_id);
    if (model == nullptr || !model->is_enabled()) {
      if (this->tune_slot_.load(std::memory_order_relaxed) == static_cast<int8_t>(i))
        this->end_tune_(false);
      ESP_LOGI(TAG, "Adopting Home Assistant change: slot %u (\"%s\") switched off", i, s.word.c_str());
      this->unload_slot_(s);
      s.spec.clear();
      s.state = SLOT_READY;
      s.error = ERR_NONE;
      s.cutoff = 0;
      this->save_slot_(i);
    }
  }

  for (auto *model : this->mww_->get_wake_words()) {
    if (!model->is_enabled() || in_slot(model->get_id()))
      continue;
    // Only a truly empty slot: one waiting on a URL is a word the customer chose, and Home
    // Assistant switching a third word on must not overwrite it.
    int free_at = -1;
    for (uint8_t i = 0; i < WL_SLOTS; i++) {
      if (this->slots_[i].spec.empty() && this->slots_[i].model_id.empty()) {
        free_at = i;
        break;
      }
    }
    if (free_at < 0) {
      ESP_LOGW(TAG, "Home Assistant enabled \"%s\" but both wake word slots are taken; disabling it",
               model->get_wake_word().c_str());
      model->disable();
      continue;
    }
    ESP_LOGI(TAG, "Adopting Home Assistant change: \"%s\" into slot %d", model->get_wake_word().c_str(), free_at);
    Slot &s = this->slots_[free_at];
    s.spec = model->get_id();
    s.model_id = model->get_id();
    s.word = model->get_wake_word();
    s.state = SLOT_READY;
    s.error = ERR_NONE;
    this->save_slot_(static_cast<uint8_t>(free_at));
  }

  this->publish_view_();
}

void MwwRuntimeLoader::save_slot_(uint8_t i) {
  SlotPrefData d{};
  strncpy(d.spec, this->slots_[i].spec.c_str(), WL_SPEC_MAX - 1);
  d.cutoff = this->slots_[i].cutoff;
  d.used = 1;
  this->prefs_[i].save(&d);
}

void MwwRuntimeLoader::publish_view_() {
  LockGuard guard{this->view_lock_};
  for (uint8_t i = 0; i < WL_SLOTS; i++) {
    const Slot &s = this->slots_[i];
    SlotViewStore &v = this->view_[i];
    // snprintf rather than strcpy for the two display fields, whose arrays are truncations by
    // design; the spec always fits, since everything writing a Slot bounds it at WL_SPEC_MAX.
    snprintf(v.spec, sizeof(v.spec), "%s", !s.pending.empty() ? s.pending.c_str() : s.spec.c_str());
    snprintf(v.word, sizeof(v.word), "%s", s.word.c_str());
    snprintf(v.id, sizeof(v.id), "%s", s.model_id.c_str());
    v.state = s.state;
    v.error = s.error;
    v.cutoff = s.cutoff;
    v.runtime = s.model_id == runtime_id_(i) || (s.model_id.empty() && is_url_(v.spec));
  }
}

void MwwRuntimeLoader::dump_config() {
  ESP_LOGCONFIG(TAG, "microWakeWord runtime loader:");
  for (uint8_t i = 0; i < WL_SLOTS; i++) {
    const Slot &s = this->slots_[i];
    ESP_LOGCONFIG(TAG, "  Slot %u: %s (state %u)", i, s.spec.empty() ? "(none)" : s.spec.c_str(), s.state);
  }
}

}  // namespace mww_runtime_loader
}  // namespace esphome

#endif  // USE_ESP32
