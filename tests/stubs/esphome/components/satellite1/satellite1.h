#pragma once
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
namespace esphome::satellite1 {
class Satellite1 {
 public:
  struct MuteState {
    bool muted{false};
    bool effectively_muted(uint32_t) const { return this->muted; }
  };
  MuteState &microphone_mute() { return this->mute; }
  MuteState mute;
  bool ready{true};
  bool responding{true};
  unsigned transfers{0};
  uint32_t frame{1};
  uint8_t angle{6};
  bool capability_present{false};
  bool hints_capability{true};
  bool valid_capability{true};
  bool write_ok{true};
  unsigned capability_reads{0};
  unsigned write_attempts{0};
  std::vector<uint8_t> hint_writes;
  bool audio_available() const { return this->ready; }
  bool transfer(uint8_t resource, uint8_t command, uint8_t *data, uint8_t length, bool *, bool retry) {
    assert(resource == 233 && !retry);
    ++this->transfers;
    if (command == 0x83) {
      assert(length == 8);
      ++this->capability_reads;
      if (!this->responding || !this->capability_present)
        return false;
      std::memset(data, 0, length);
      data[0] = this->valid_capability ? 'S' : 'X';
      data[1] = '1'; data[2] = 'D'; data[3] = 1;
      data[4] = this->hints_capability ? 1 : 0;
      return true;
    }
    if (command == 4) {
      assert(length == 1);
      ++this->write_attempts;
      if (!this->responding || !this->write_ok)
        return false;
      this->hint_writes.push_back(data[0]);
      return true;
    }
    assert(command == 0x80 && length == 32);
    if (!this->responding)
      return false;
    std::memset(data, 0, length);
    data[2] = 40;
    data[3] = 3;
    data[14] = this->angle;
    for (int i = 0; i < 4; ++i)
      data[8 + i] = this->frame >> (8 * i);
    return true;
  }
};
}  // namespace esphome::satellite1
