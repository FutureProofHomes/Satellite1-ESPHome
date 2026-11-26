#include "directional_leds.h"
#include "esphome/core/log.h"

namespace esphome::satellite1_directional_leds {

static const char *const TAG = "satellite1.directional_leds";

void DirectionalLEDs::dump_config() {
  ESP_LOGCONFIG(TAG, "Optional directional voice LEDs (XMOS direction telemetry required)");
}

void DirectionalLEDs::update_meter_() {
  this->speaker_->set_level_meter_enabled(this->enabled_ && this->replying_ && this->available_ &&
                                          this->have_reply_direction_);
}

void DirectionalLEDs::set_available_(bool available) {
  if (this->available_ == available)
    return;
  this->available_ = available;
  ESP_LOGD(TAG, "Direction telemetry %s (listening=%u, replying=%u)", available ? "available" : "unavailable",
           this->listening_, this->replying_);
  this->update_meter_();
  this->availability_callback_.call();
}

void DirectionalLEDs::set_enabled(bool enabled) {
  if (this->enabled_ == enabled)
    return;
  this->enabled_ = enabled;
  this->was_ready_ = false;
  this->direction_.reset();
  this->have_reply_direction_ = false;
  this->set_available_(false);
  this->update_meter_();
}

void DirectionalLEDs::set_phase(bool listening, bool replying) {
  const bool starting = listening && !this->listening_;
  const bool finishing = !listening && this->listening_;
  this->listening_ = listening;
  this->replying_ = replying;
  if (starting) {
    this->direction_.begin_listening();
    this->have_reply_direction_ = false;
    this->listening_started_at_ = millis();
    this->next_poll_ = this->listening_started_at_;
  } else if (finishing) {
    this->have_reply_direction_ = this->direction_.has_remembered_direction();
    this->reply_position_ = this->direction_.dominant();
  }
  this->update_meter_();
}

bool DirectionalLEDs::telemetry_fresh_(uint32_t now) const {
  // Polling pauses between conversations. A previously working connection gets
  // one bounded window to resume, without claiming support on an unprobed image.
  return this->direction_.progressing(now) ||
         (this->available_ && this->listening_ && now - this->listening_started_at_ <= DirectionState::PROGRESS_TIMEOUT_MS);
}

void DirectionalLEDs::loop() {
  const bool ready = this->enabled_ && this->satellite_->audio_available() &&
                     !this->satellite_->microphone_mute().effectively_muted(millis());
  if (!ready) {
    if (this->was_ready_) {
      this->was_ready_ = false;
      this->have_reply_direction_ = false;
      this->direction_.reset();
      this->set_available_(false);
    }
    return;
  }
  const uint32_t now = millis();
  if (!this->was_ready_) {
    this->was_ready_ = true;
    this->supported_ = false;
    this->probe_failures_ = 0;
    this->direction_.reset();
    this->next_poll_ = now;
  }
  // Unsupported images are probed at most three times per connection/enable.
  // A known supported image can recover after transient transport/DSP stalls.
  if ((!this->supported_ && this->probe_failures_ >= 3) || (!this->listening_ && this->available_) ||
      int32_t(now - this->next_poll_) < 0)
    return;
  uint8_t payload[32]{};
  const bool received = this->satellite_->transfer(233, 0x80, payload, sizeof(payload), nullptr, false);
  if (!received || (payload[3] & 2) == 0 || payload[14] >= DirectionState::LED_COUNT) {
    if (!this->supported_)
      ++this->probe_failures_;
    // Non-retrying SPI reads can miss while XMOS is busy. Keep the renderer on
    // the directional effect while recent frame progress still proves liveness.
    // Unknown firmware keeps the slower, bounded capability probe.
    this->next_poll_ = now + (this->supported_ ? 50 : 1000);
    this->set_available_(this->supported_ && this->telemetry_fresh_(now));
    if (this->probe_failures_ == 3)
      ESP_LOGI(TAG, "XMOS direction telemetry unavailable; using standard animations");
    return;
  }
  this->supported_ = true;
  this->probe_failures_ = 0;
  this->next_poll_ = now + 50;
  this->direction_.observe(payload, now, this->listening_);
  this->set_available_(this->telemetry_fresh_(now));
}

}  // namespace esphome::satellite1_directional_leds
