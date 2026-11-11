#pragma once

// The XMOS speaker EQ's device-control resource, as Satellite1-XMOS defines it in
// src/speaker_eq/speaker_eq_cmds.h and documents in docs/device-control-spi-protocol.md; change both
// together. Every multi-byte field is little endian and every float is IEEE-754 binary32. Payload
// lengths are exact: the XMOS rejects any other length.

#include <cstdint>

namespace esphome::satellite1_dsp::eq {

static const uint8_t RESID = 231;

static const uint8_t API_MAJOR = 1;

static const uint8_t CMD_GET_INFO = 0;     // read, INFO_BYTES
static const uint8_t CMD_GET_STATS = 1;    // read, STATS_BYTES
static const uint8_t CMD_SET_ROUTING = 2;  // write, ROUTING_BYTES
static const uint8_t CMD_SET_SLOT = 3;     // write, SLOT_BYTES
static const uint8_t CMD_GET_SLOT = 4;     // read, SLOT_BYTES

// SET_ROUTING: u8 eq_enabled (0 or 1), u8 profile_id.
static const uint8_t ROUTING_BYTES = 2;

enum ProfileId : uint8_t {
  PROFILE_NONE = 0,
  PROFILE_SPEAKER = 1,
  PROFILE_LINE_OUT = 2,
};

static const uint8_t NUM_SLOTS = 2;
static const uint8_t MAX_FILTERS = 14;
static const uint8_t MAX_SECTIONS = 16;

// Slot payload: u8 slot, f32 preamp_db, f32 limiter_db, f32 attack_ms, f32 release_ms,
// u8 filter_count, then MAX_FILTERS records of u8 type, u8 flags, f32 freq_hz, f32 gain_db, f32 q.
static const uint8_t FILTER_RECORD_BYTES = 14;
static const uint8_t SLOT_HEADER_BYTES = 18;
static const uint8_t SLOT_BYTES = SLOT_HEADER_BYTES + MAX_FILTERS * FILTER_RECORD_BYTES;
static const uint8_t FILTER_FLAG_ENABLED = 0x01;

static const uint8_t INFO_BYTES = 40;
static const uint8_t STATS_BYTES = 48;

enum FilterType : uint8_t {
  FILTER_NONE = 0,
  FILTER_PEAK = 1,
  FILTER_LOW_SHELF = 2,
  FILTER_HIGH_SHELF = 3,
  FILTER_HIGHPASS_Q = 4,  // 12 dB/oct, custom Q
  FILTER_LOWPASS_Q = 5,   // 12 dB/oct, custom Q
  FILTER_HIGHPASS_BW12 = 6,
  FILTER_HIGHPASS_BW24 = 7,
  FILTER_HIGHPASS_LR12 = 8,
  FILTER_HIGHPASS_LR24 = 9,
  FILTER_LOWPASS_BW12 = 10,
  FILTER_LOWPASS_BW24 = 11,
  FILTER_LOWPASS_LR12 = 12,
  FILTER_LOWPASS_LR24 = 13,
  NUM_FILTER_TYPES
};

inline bool filter_uses_gain(uint8_t type) {
  return type == FILTER_PEAK || type == FILTER_LOW_SHELF || type == FILTER_HIGH_SHELF;
}

inline bool filter_uses_q(uint8_t type) {
  return filter_uses_gain(type) || type == FILTER_HIGHPASS_Q || type == FILTER_LOWPASS_Q;
}

// Biquad sections a filter costs on the XMOS, out of MAX_SECTIONS. Disabled filters, and peaks or
// shelves under 0.01 dB, cost none (but are still range-checked).
inline uint8_t filter_sections(uint8_t type, bool enabled, float gain_db) {
  if (type == FILTER_NONE || type >= NUM_FILTER_TYPES || !enabled)
    return 0;
  if (filter_uses_gain(type) && gain_db > -0.01f && gain_db < 0.01f)
    return 0;
  switch (type) {
    case FILTER_HIGHPASS_BW24:
    case FILTER_HIGHPASS_LR24:
    case FILTER_LOWPASS_BW24:
    case FILTER_LOWPASS_LR24:
      return 2;
    default:
      return 1;
  }
}

enum BypassReason : uint8_t {
  BYPASS_NONE = 0,
  BYPASS_DISABLED = 1,
  BYPASS_NO_PROFILE = 2,
  BYPASS_FLAT = 3,
  BYPASS_OVERLOAD = 4,
};

enum SlotStatus : uint8_t {
  SLOT_EMPTY = 0,
  SLOT_OK = 1,
  SLOT_REJECTED_HEADROOM = 2,
  SLOT_PENDING = 3,
};

// Range limits the XMOS enforces; anything outside fails the whole SET_SLOT.
static constexpr float PREAMP_MIN_DB = -24.0f, PREAMP_MAX_DB = 12.0f;
static constexpr float LIMITER_MIN_DB = -12.0f, LIMITER_MAX_DB = 0.0f;
static constexpr float ATTACK_MIN_MS = 0.1f, ATTACK_MAX_MS = 100.0f;
static constexpr float RELEASE_MIN_MS = 5.0f, RELEASE_MAX_MS = 5000.0f;
static constexpr float FREQ_MIN_HZ = 20.0f, FREQ_MAX_HZ = 20000.0f;
static constexpr float GAIN_MIN_DB = -30.0f, GAIN_MAX_DB = 12.0f;
static constexpr float Q_MIN = 0.1f, Q_MAX = 10.0f;
// The worst gain the XMOS accepts at any point in the filter chain.
static constexpr float HEADROOM_CAP_DB = 18.0f;

}  // namespace esphome::satellite1_dsp::eq
