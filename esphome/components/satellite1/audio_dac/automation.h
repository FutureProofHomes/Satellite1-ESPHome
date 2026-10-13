#pragma once

#include "esphome/core/automation.h"
#include "dac_proxy.h"

namespace esphome {

namespace satellite1 {

template<typename... Ts> class ActivateAction : public Action<Ts...>, public Parented<DACProxy> {
 public:
  void play(const Ts &...x) override { this->parent_->activate(); }
};

template<typename... Ts> class ActivateLineOutAction : public Action<Ts...>, public Parented<DACProxy> {
 public:
  void play(const Ts &...x) override { this->parent_->activate_line_out(); }
};

template<typename... Ts> class ActivateSpeakerAction : public Action<Ts...>, public Parented<DACProxy> {
 public:
  void play(const Ts &...x) override { this->parent_->activate_speaker(); }
};

template<typename... Ts> class SetVolumeLimitsAction : public Action<Ts...>, public Parented<DACProxy> {
 public:
  TEMPLATABLE_VALUE(DacOutput, output)
  TEMPLATABLE_VALUE(float, min_db)
  TEMPLATABLE_VALUE(float, max_db)

  void play(const Ts &...x) override {
    this->parent_->set_volume_limits(this->output_.value(x...), this->min_db_.value(x...), this->max_db_.value(x...));
  }
};

class StateTrigger : public Trigger<> {
 public:
  explicit StateTrigger(DACProxy *proxy) {
    proxy->add_on_state_callback([this]() { this->trigger(); });
  }
};

template<DacOutput output_dac> class DACProxyStateTrigger : public Trigger<> {
 public:
  explicit DACProxyStateTrigger(DACProxy *proxy) {
    proxy->add_on_selection_callback([this, proxy]() {
      if (proxy->selected_output() == output_dac)
        this->trigger();
    });
  }
};

using SpeakerActivatedTrigger = DACProxyStateTrigger<SPEAKER>;
using LineOutActivatedTrigger = DACProxyStateTrigger<LINE_OUT>;

}  // namespace satellite1
}  // namespace esphome
