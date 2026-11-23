// SPDX-License-Identifier: GPL-3.0-only
#include "wyoming_client.h"
#include <chrono>
#include <iostream>
#include <vector>

using namespace esphome::stt_wake_verifier;

uint32_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char **argv) {
  WyomingEndpoint endpoint;
  if (argc < 3 || !parse_wyoming_endpoint(argv[2], endpoint))
    return 2;
  if (std::string(argv[1]) == "parse") {
    std::cout << endpoint.host << '\n' << endpoint.port << '\n';
    return 0;
  }
  if (argc != 5)
    return 2;
  std::vector<uint8_t> pcm(std::stoul(argv[4]));
  for (size_t i = 0; i < pcm.size(); ++i)
    pcm[i] = i % 251;
  const auto result = transcribe_wyoming(endpoint, pcm.data(), pcm.size(), now_ms(), std::stoul(argv[3]), now_ms);
  std::cout << (result.error.empty() ? "ok\n" : "error\n") << (result.error.empty() ? result.text : result.error);
}
