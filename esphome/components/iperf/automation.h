#pragma once

#include "esphome/core/automation.h"
#include "idf_iperf.h"

namespace esphome {
namespace iperf {

template<typename... Ts> class IPerfStartClientAction : public Action<Ts...>, public Parented<Iperf> {
 public:
  TEMPLATABLE_VALUE(std::string, server)
  TEMPLATABLE_VALUE(uint32_t, duration)

  void play(const Ts &... x) override {
    if (this->server_.has_value()) {
      this->parent_->set_remote_ip(this->server_.value(x...));
    }
    if (this->duration_.has_value()) {
      this->parent_->set_duration(this->duration_.value(x...));
    }
    this->parent_->start_client();
  }
};

template<typename... Ts> class IPerfStartServerAction : public Action<Ts...>, public Parented<Iperf> {
 public:
  void play(const Ts &...) override { this->parent_->start_server(); }
};

template<typename... Ts> class IPerfStopAction : public Action<Ts...>, public Parented<Iperf> {
 public:
  void play(const Ts &...) override { this->parent_->stop(); }
};

}  // namespace iperf
}  // namespace esphome
