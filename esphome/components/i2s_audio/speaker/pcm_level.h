#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::i2s_audio {

// Read-only mean absolute PCM level. Sample every eighth frame, including both
// stereo channels, without allocation or modifying the audio buffer.
inline uint32_t pcm_level_q16(const uint8_t *data, size_t length, uint8_t bytes_per_sample, uint8_t channels) {
  if (bytes_per_sample == 0 || bytes_per_sample > 4 || channels == 0)
    return 0;
  const size_t frame_bytes = bytes_per_sample * channels;
  uint64_t total = 0;
  uint32_t count = 0;
  const uint8_t bits = bytes_per_sample * 8;
  for (size_t offset = 0; offset + frame_bytes <= length; offset += frame_bytes * 8) {
    for (uint8_t channel = 0; channel < channels; ++channel) {
      uint32_t raw = 0;
      for (uint8_t byte = 0; byte < bytes_per_sample; ++byte)
        raw |= uint32_t(data[offset + channel * bytes_per_sample + byte]) << (byte * 8);
      const int64_t sample = (raw & (uint32_t(1) << (bits - 1))) ? int64_t(raw) - (int64_t(1) << bits) : raw;
      total += sample < 0 ? -sample : sample;
      ++count;
    }
  }
  return count == 0 ? 0 : uint32_t((total * 65535) / (uint64_t(count) * (uint64_t(1) << (bits - 1))));
}

}  // namespace esphome::i2s_audio
