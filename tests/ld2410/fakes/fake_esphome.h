#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace esphome {
inline uint32_t fake_now = 100;
inline uint32_t millis() { return fake_now; }
inline uint32_t fnv1_hash(const char *) { return 1; }
enum EntityCategory { ENTITY_CATEGORY_NONE, ENTITY_CATEGORY_DIAGNOSTIC };
constexpr unsigned ENTITY_FIELD_DC_SHIFT = 0, ENTITY_FIELD_UOM_SHIFT = 4, ENTITY_FIELD_ICON_SHIFT = 8,
                   ENTITY_FIELD_INTERNAL_SHIFT = 16, ENTITY_FIELD_DISABLED_BY_DEFAULT_SHIFT = 17,
                   ENTITY_FIELD_ENTITY_CATEGORY_SHIFT = 18;
class EntityBase {
 protected:
  void configure_entity_(const char *, uint32_t, uint32_t) {}
};
class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void dump_config() {}
};
namespace sensor {
enum StateClass { STATE_CLASS_NONE, STATE_CLASS_MEASUREMENT };
class Sensor : public EntityBase {
 public:
  float state = 0;
  unsigned publishes = 0;
  void publish_state(float value) {
    state = value;
    ++publishes;
  }
  void set_state_class(StateClass) {}
  void set_accuracy_decimals(int8_t) {}
};
}  // namespace sensor
namespace binary_sensor {
class BinarySensor : public EntityBase {
 public:
  bool state = false;
  unsigned publishes = 0;
  void publish_state(bool value) {
    state = value;
    ++publishes;
  }
};
}  // namespace binary_sensor
namespace text_sensor {
class TextSensor : public EntityBase {
 public:
  std::string state;
  unsigned publishes = 0;
  void publish_state(const std::string &value) {
    state = value;
    ++publishes;
  }
};
}  // namespace text_sensor
namespace switch_ {
class Switch : public EntityBase {
 public:
  void publish_state(bool) {}

 protected:
  virtual void write_state(bool) = 0;
};
}  // namespace switch_
namespace button {
class Button : public EntityBase {
 public:
  void add_on_press_callback(std::function<void()>) {}

 protected:
  virtual void press_action() = 0;
};
}  // namespace button
namespace uart {
class UARTDevice {
 public:
  std::deque<uint8_t> rx;
  std::vector<std::vector<uint8_t>> writes;
  size_t reads = 0;
  size_t available() const { return rx.size(); }
  bool read_byte(uint8_t *byte) {
    if (rx.empty())
      return false;
    *byte = rx.front();
    rx.pop_front();
    ++reads;
    return true;
  }
  void write_array(const uint8_t *data, size_t size) { writes.emplace_back(data, data + size); }
  void inject(const std::vector<uint8_t> &data) { rx.insert(rx.end(), data.begin(), data.end()); }
};
}  // namespace uart
struct PreferenceState {
  unsigned saves = 0, syncs = 0;
  std::vector<uint8_t> stored;
};
inline PreferenceState preference_state;
class ESPPreferenceObject {
 public:
  template<typename T> bool load(T *) { return false; }
  template<typename T> bool save(const T *value) {
    ++preference_state.saves;
    const auto *bytes = reinterpret_cast<const uint8_t *>(value);
    preference_state.stored.assign(bytes, bytes + sizeof(T));
    return true;
  }
};
class ESPPreferences {
 public:
  template<typename T> ESPPreferenceObject make_preference(uint32_t) { return {}; }
  bool sync() {
    ++preference_state.syncs;
    return true;
  }
};
inline ESPPreferences fake_preferences;
inline ESPPreferences *global_preferences = &fake_preferences;
class Application {
 public:
  void register_binary_sensor(binary_sensor::BinarySensor *) {}
  void register_text_sensor(text_sensor::TextSensor *) {}
  void register_sensor(sensor::Sensor *) {}
  void register_button(button::Button *) {}
};
inline Application App;
}  // namespace esphome

#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGW(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGV(...) ((void) 0)
