#include "audio_hints.h"
#include "esphome/core/log.h"

namespace esphome::satellite1_audio_hints {
static const char *const TAG = "satellite1.audio_hints";
static constexpr uint8_t RESOURCE = 233;
static constexpr uint8_t READ_CAPABILITIES = 0x83;
static constexpr uint8_t SET_HOST_HINTS = 4;
static constexpr uint8_t PLAYBACK_ACTIVE = 1;
static constexpr uint8_t VOICE_ACTIVE = 8;

void AudioHints::dump_config() { ESP_LOGCONFIG(TAG, "Optional XMOS playback/listening hints (capability-gated)"); }

void AudioHints::loop() {
  const bool ready = this->enabled_ && this->satellite_->audio_available();
  if (!ready) {
    this->speaker_->set_level_meter_enabled(false, i2s_audio::PlaybackLevelUser::AUDIO_HINTS);
    this->was_ready_ = this->supported_ = this->probe_finished_ = this->sent_ = false;
    return;
  }
  const uint32_t now = millis();
  if (!this->was_ready_) {
    this->was_ready_ = true;
    this->probe_failures_ = 0;
    this->next_poll_ = now;
  }
  if (int32_t(now - this->next_poll_) < 0)
    return;
  this->next_poll_ = now + 125;
  if (!this->probe_finished_) {
    uint8_t capability[8]{};
    const bool received =
        this->satellite_->transfer(RESOURCE, READ_CAPABILITIES, capability, sizeof(capability), nullptr, false);
    if (!received || capability[0] != 'S' || capability[1] != '1' || capability[2] != 'D' || capability[3] != 1) {
      this->probe_finished_ = ++this->probe_failures_ >= 3;
      this->next_poll_ = now + 1000;
      return;
    }
    this->probe_finished_ = true;
    this->supported_ = (capability[4] & 1) != 0;
    this->speaker_->set_level_meter_enabled(this->supported_, i2s_audio::PlaybackLevelUser::AUDIO_HINTS);
    ESP_LOGD(TAG, "XMOS status hints %s", this->supported_ ? "supported" : "unavailable");
    return;
  }
  if (!this->supported_)
    return;
  uint8_t flags = this->speaker_->playback_level() >= 0.004f ? PLAYBACK_ACTIVE : 0;
  if (!this->satellite_->microphone_mute().effectively_muted(now) && this->voice_active_ && this->voice_active_())
    flags |= VOICE_ACTIVE;
  if (this->sent_ && flags == this->last_flags_ && now - this->last_sent_at_ < 1000)
    return;
  // Non-retrying writes are bounded to one per poll. XMOS expires stale hints
  // if the host disappears. The LED switch does not own this component.
  if (this->satellite_->transfer(RESOURCE, SET_HOST_HINTS, &flags, sizeof(flags), nullptr, false)) {
    this->last_flags_ = flags;
    this->last_sent_at_ = now;
    this->sent_ = true;
  }
}
}  // namespace esphome::satellite1_audio_hints
