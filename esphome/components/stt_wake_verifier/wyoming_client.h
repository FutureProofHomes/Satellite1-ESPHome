// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome::stt_wake_verifier {

struct WyomingEndpoint {
  std::string host;
  uint16_t port{0};
};

struct WyomingTranscript {
  std::string text, error;
};

// Accept host:port, [IPv6]:port, and optional tcp:// or wyoming:// prefixes.
bool parse_wyoming_endpoint(const std::string &value, WyomingEndpoint &endpoint);

// A single bounded Wyoming TCP exchange. Called only by the STT worker.
WyomingTranscript transcribe_wyoming(const WyomingEndpoint &endpoint, const uint8_t *pcm, size_t pcm_size,
                                     uint32_t started, uint32_t timeout, uint32_t (*clock)());

}  // namespace esphome::stt_wake_verifier
