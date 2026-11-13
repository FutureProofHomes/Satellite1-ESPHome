#pragma once

#include "esphome/core/helpers.h"

#include <cmath>
#include <utility>

namespace esphome::speaker_source {

struct VolumeMuteState {
  float volume;
  bool muted;

  bool operator==(const VolumeMuteState &other) const {
    return this->volume == other.volume && this->muted == other.muted;
  }
  bool operator!=(const VolumeMuteState &other) const { return !(*this == other); }
};

// Providers accept intents and publish committed pairs on the main loop only.
// initialize() may defer publication until the provider's backing state is ready.
class VolumeState {
 public:
  virtual ~VolumeState() = default;
  virtual void initialize() = 0;
  virtual bool set_volume(float volume) = 0;
  virtual bool set_muted(bool muted) = 0;

  bool is_initialized() const { return this->initialized_; }
  VolumeMuteState state() const { return this->state_; }
  bool is_publishing() const { return this->publishing_; }

  template<typename F> void add_on_state_callback(F &&callback) {
    this->state_callbacks_.add(std::forward<F>(callback));
  }

 protected:
  bool publish_state_(float volume, bool muted, bool force = false) {
    if (!std::isfinite(volume) || volume < 0.0f || volume > 1.0f) {
      return false;
    }
    if (!force && this->initialized_ && this->state_.volume == volume && this->state_.muted == muted) {
      return false;
    }
    this->state_ = {volume, muted};
    this->initialized_ = true;
    this->pending_publication_ = true;
    this->publish_pending_();
    return true;
  }

  void publish_pending_() {
    if (this->publishing_) {
      return;
    }
    // Finish each snapshot's delivery before publishing intents accepted by its callbacks.
    this->publishing_ = true;
    while (this->pending_publication_) {
      this->pending_publication_ = false;
      const auto current = this->state_;
      this->state_callbacks_.call(current);
    }
    this->publishing_ = false;
  }

  bool initialized_{false};
  bool publishing_{false};
  bool pending_publication_{false};
  VolumeMuteState state_{0.0f, false};
  CallbackManager<void(const VolumeMuteState &)> state_callbacks_;
};

}  // namespace esphome::speaker_source
