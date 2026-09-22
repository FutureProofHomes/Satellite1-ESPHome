#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <esp_heap_caps.h>

namespace esphome {
namespace satellite1_radar {

/// Places everything this component heap-allocates at detect time in PSRAM.
///
/// The radar handlers and their runtime entities are `new`ed once a module answers on the UART and
/// then live for the life of the device. extram_bss cannot reach them - it only moves the
/// generated main.cpp statics - so without this they were several KB of internal heap: the LD2410
/// handler alone is ~3KB (its command queue is 32 x 72 bytes), and each runtime entity object a few
/// hundred more. Everything that touches these objects runs on the main loop or the httpd task,
/// with the cache enabled, so PSRAM placement is safe; nothing here is read from an ISR.
///
/// An empty mixin base rather than call-site allocators, so `new` and every std::unique_ptr keep
/// working unchanged: class-scoped operator new is what the language already provides for exactly
/// this. The internal-heap fallback keeps a PSRAM-less board booting; operator delete uses free(),
/// which on ESP-IDF routes any pointer back to the heap that owns it.
struct PsramAllocated {
  static void *operator new(size_t size) {
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == nullptr)
      p = ::operator new(size);  // Internal heap; aborts on exhaustion exactly as any other new.
    return p;
  }
  static void operator delete(void *p) { free(p); }
};

struct DeviceClassMeta {
  uint8_t distance{0};
  uint8_t illuminance{0};
  uint8_t occupancy{0};
  uint8_t motion{0};
};

struct UnitMeta {
  uint8_t centimeter{0};
  uint8_t percent{0};
};

struct IconMeta {
  uint8_t radar{0};
  uint8_t chip{0};
  uint8_t signal{0};
  uint8_t motion_sensor{0};
  uint8_t account_multiple{0};
  uint8_t account{0};
  uint8_t account_arrow_right{0};
  uint8_t tune_vertical{0};
  uint8_t factory{0};
  uint8_t restart{0};
  uint8_t database_refresh{0};
};

inline uint32_t pack_entity_fields(uint8_t device_class_idx, uint8_t uom_idx, uint8_t icon_idx, bool internal,
                                   bool disabled_by_default, EntityCategory entity_category) {
  return (static_cast<uint32_t>(device_class_idx) << ENTITY_FIELD_DC_SHIFT) |
         (static_cast<uint32_t>(uom_idx) << ENTITY_FIELD_UOM_SHIFT) |
         (static_cast<uint32_t>(icon_idx) << ENTITY_FIELD_ICON_SHIFT) |
         (static_cast<uint32_t>(internal) << ENTITY_FIELD_INTERNAL_SHIFT) |
         (static_cast<uint32_t>(disabled_by_default) << ENTITY_FIELD_DISABLED_BY_DEFAULT_SHIFT) |
         (static_cast<uint32_t>(entity_category) << ENTITY_FIELD_ENTITY_CATEGORY_SHIFT);
}

/// Runtime show/hide for the frontends, written against the protected flag rather than
/// EntityBase::set_internal because the latter is deprecated for exactly the failure this
/// component avoids: a bare flip notifies nobody. Here every flip is followed by a Home
/// Assistant config-entry reload (LD2450Handler::sync_entity_layout_) that re-runs
/// ListEntities - the missing notification - and both the API server and web_server test
/// is_internal() at publish time, so state traffic stops and starts with the flag.
#define SAT1_RADAR_DYNAMIC_VISIBILITY \
  void set_frontend_hidden(bool hidden) { this->flags_.internal = hidden; }

class Satellite1RadarDynamicSensor : public sensor::Sensor, public PsramAllocated {
 public:
  SAT1_RADAR_DYNAMIC_VISIBILITY
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t device_class_idx = 0, uint8_t uom_idx = 0,
                         uint8_t icon_idx = 0, bool has_state_class = false,
                         sensor::StateClass state_class = sensor::STATE_CLASS_NONE, int8_t accuracy_decimals = -1) {
    this->configure_entity_(
        name, 0, pack_entity_fields(device_class_idx, uom_idx, icon_idx, false, disabled_by_default, entity_category));
    if (has_state_class)
      this->set_state_class(state_class);
    if (accuracy_decimals >= 0)
      this->set_accuracy_decimals(accuracy_decimals);
  }
};

class Satellite1RadarDynamicBinarySensor : public binary_sensor::BinarySensor, public PsramAllocated {
 public:
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t device_class_idx = 0, uint8_t icon_idx = 0) {
    this->configure_entity_(
        name, 0, pack_entity_fields(device_class_idx, 0, icon_idx, false, disabled_by_default, entity_category));
  }
};

class Satellite1RadarDynamicTextSensor : public text_sensor::TextSensor, public PsramAllocated {
 public:
  SAT1_RADAR_DYNAMIC_VISIBILITY
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t icon_idx = 0) {
    this->configure_entity_(name, 0, pack_entity_fields(0, 0, icon_idx, false, disabled_by_default, entity_category));
  }
};

class Satellite1RadarButton : public button::Button, public Component, public PsramAllocated {
 public:
  void setup() override {}
  void dump_config() override {}
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t icon_idx = 0) {
    this->configure_entity_(name, 0, pack_entity_fields(0, 0, icon_idx, false, disabled_by_default, entity_category));
  }

 protected:
  void press_action() override {}
};

}  // namespace satellite1_radar
}  // namespace esphome
