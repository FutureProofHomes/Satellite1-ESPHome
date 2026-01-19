#pragma once

#include "esphome/core/automation.h"
#include "snapcast_client.h"

namespace esphome {
namespace snapcast {

template< typename... Ts>
class EnableAction : public Action<Ts...>, public Parented<SnapcastClient> {
 public:
  void play(const Ts &...x) override { this->parent_->enable(); }
};

template< typename... Ts>
class DisableAction : public Action<Ts...>, public Parented<SnapcastClient> {
 public:
  void play(const Ts &...x) override { this->parent_->disable(); }
};


}
}