#pragma once
#include <functional>
#include <utility>
#include <vector>
namespace esphome {
template<typename... T> class Trigger {
 public:
  void trigger(T...) {}
};
template<typename T> class CallbackManager;
template<> class CallbackManager<void()> {
 public:
  template<typename F> void add(F &&callback) { this->callbacks_.emplace_back(std::forward<F>(callback)); }
  void call() {
    for (auto &callback : this->callbacks_)
      callback();
  }

 private:
  std::vector<std::function<void()>> callbacks_;
};
}  // namespace esphome
