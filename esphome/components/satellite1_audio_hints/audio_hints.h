#pragma once
#include <functional>
#include "esphome/components/i2s_audio/speaker/i2s_audio_speaker.h"
#include "esphome/components/satellite1/satellite1.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome::satellite1_audio_hints {
// Observe existing states; do not own voice-session or playback transitions.
class AudioHints : public Component {
 public:
  void dump_config() override;
  void loop() override;
  void set_satellite(satellite1::Satellite1 *satellite) { this->satellite_ = satellite; }
  void set_speaker(i2s_audio::I2SAudioSpeaker *speaker) { this->speaker_ = speaker; }
  void set_voice_active(std::function<bool()> voice_active) { this->voice_active_ = std::move(voice_active); }
  void set_enabled(bool enabled) { this->enabled_ = enabled; }
  bool supported() const { return this->supported_; }

 protected:
  satellite1::Satellite1 *satellite_{nullptr};
  i2s_audio::I2SAudioSpeaker *speaker_{nullptr};
  std::function<bool()> voice_active_;
  uint32_t next_poll_{0};
  uint32_t last_sent_at_{0};
  uint8_t probe_failures_{0};
  uint8_t last_flags_{0};
  bool was_ready_{false};
  bool enabled_{false};
  bool supported_{false};
  bool probe_finished_{false};
  bool sent_{false};
};
}  // namespace esphome::satellite1_audio_hints
