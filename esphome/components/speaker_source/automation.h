#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_SPEAKER_SOURCE_MEDIA_PLAYER)

#include "esphome/core/automation.h"
#include "speaker_source_media_player.h"

namespace esphome::speaker_source {

template<typename... Ts> class SetPlaylistDelayAction final : public Action<Ts...> {
 public:
  explicit SetPlaylistDelayAction(SpeakerSourceMediaPlayer *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, pipeline)
  TEMPLATABLE_VALUE(uint32_t, delay)

  void play(const Ts &...x) override {
    this->parent_->set_playlist_delay_ms(this->pipeline_.value(x...), this->delay_.value(x...));
  }

 protected:
  SpeakerSourceMediaPlayer *parent_;
};

}  // namespace esphome::speaker_source

#endif  // USE_ESP32
