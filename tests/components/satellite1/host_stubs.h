#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace host {
inline uint32_t now = 0;
inline std::optional<uint32_t> loop_start_time;
inline unsigned spi_calls = 0;
}  // namespace host

inline void vTaskDelay(unsigned) {}

#define ESP_LOGCONFIG(...) ((void) 0)
#define ESP_LOGW(...) ((void) 0)
#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)

namespace esphome {
class Application {
 public:
  uint32_t get_loop_component_start_time() const { return host::loop_start_time.value_or(host::now); }
};
inline Application App;
inline uint32_t millis() { return host::now; }
inline void delay(uint32_t ms) { host::now += ms; }
namespace setup_priority {
constexpr float IO = 900;
}
class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void dump_config() {}
  virtual void loop() {}
  virtual float get_setup_priority() const { return 0; }
};
class GPIOPin {
 public:
  bool level = false;
  void setup() {}
  void digital_write(bool value) { this->level = value; }
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
template<typename T> class Parented {
 protected:
  T *parent_ = nullptr;
};
namespace spi {
enum { BIT_ORDER_MSB_FIRST, CLOCK_POLARITY_HIGH, CLOCK_PHASE_TRAILING, DATA_RATE_8MHZ };
template<int, int, int, int> class SPIDevice {
 public:
  void spi_setup() {}
  void enable() {}
  void disable() {}
  uint8_t transfer_byte(uint8_t value) { return value; }
  void transfer_array(uint8_t *data, size_t size) {
    ++host::spi_calls;
    // Valid status frame, device not ready: loop probes stop after one exchange.
    std::memset(data, 0, size);
    data[0] = 1;
  }
};
}  // namespace spi
}  // namespace esphome
