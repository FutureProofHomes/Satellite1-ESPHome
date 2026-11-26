#pragma once
#include <cstdint>
namespace esphome {
inline uint32_t test_millis = 0;
inline uint32_t millis() { return test_millis; }
}  // namespace esphome
