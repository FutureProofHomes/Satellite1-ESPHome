#include "esphome/components/satellite1_radar/ld2410_handler.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <utility>

using namespace esphome;
using satellite1_radar::LD2410Handler;
using Bytes = std::vector<uint8_t>;
using Config = LD2410Handler::LD2410BackendConfig;

class TestHandler : public LD2410Handler {
 public:
  using LD2410Handler::LD2410Handler;
  using LD2410Handler::handle_ack_frame_;
  void prepare() {
    setup();
    // setup's queries are irrelevant to receive tests, but use its real buffer/preferences initialization.
    cmd_queue_count_ = 0;
    waiting_for_ack_ = true;
    ack_wait_start_ = millis();
  }
  bool waiting() const { return waiting_for_ack_; }
  bool readback_pending() const { return bt_readback_pending_; }
};

static bool equal(const Config &a, const Config &b) {
  return a.timeout_seconds == b.timeout_seconds && a.max_move_gate == b.max_move_gate &&
         a.max_still_gate == b.max_still_gate && a.distance_resolution == b.distance_resolution &&
         a.bluetooth_enabled == b.bluetooth_enabled &&
         std::equal(a.gate_move_threshold, a.gate_move_threshold + 9, b.gate_move_threshold) &&
         std::equal(a.gate_still_threshold, a.gate_still_threshold + 9, b.gate_still_threshold);
}

struct Fixture {
  uart::UARTDevice uart;
  TestHandler handler{uart};
  text_sensor::TextSensor version;
  Config initial;
  Fixture() {
    fake_now = 100;
    preference_state = {};
    handler.prepare();
    initial.timeout_seconds = 4321;
    initial.max_move_gate = 2;
    initial.max_still_gate = 3;
    initial.distance_resolution = 1;
    initial.bluetooth_enabled = true;
    for (size_t g = 0; g < 9; ++g) {
      initial.gate_move_threshold[g] = 60 + g;
      initial.gate_still_threshold[g] = 80 + g;
    }
    assert(handler.set_backend_config(initial));
    preference_state = {};
    handler.version_text_sensor = &version;
  }
  void parse(const Bytes &frame) {
    // Exact allocation makes sanitizer checks effective for every truncated prefix.
    auto bytes = std::make_unique<uint8_t[]>(frame.size());
    std::copy(frame.begin(), frame.end(), bytes.get());
    handler.handle_ack_frame_(bytes.get(), frame.size());
  }
  void rejected(const Bytes &frame, bool failed_ack = false) {
    parse(frame);
    assert(equal(handler.get_backend_config(), initial));
    assert(preference_state.saves == 0 && preference_state.syncs == 0);
    assert(version.publishes == 0);
    assert(handler.waiting() == !failed_ack);
    assert(!handler.readback_pending());
  }
  void accepted(const Config &expected, unsigned saves = 1) {
    assert(equal(handler.get_backend_config(), expected));
    assert(preference_state.saves == saves && preference_state.syncs == saves);
    if (saves) {
      assert(preference_state.stored.size() == sizeof(Config));
      Config persisted;
      std::memcpy(&persisted, preference_state.stored.data(), sizeof(persisted));
      assert(equal(persisted, expected));
    }
    assert(!handler.waiting());
  }
  void feed(const Bytes &frame) {
    uart.inject(frame);
    handler.loop();
  }
};

static void length(Bytes &frame) {
  const size_t payload = frame.size() - 10;
  frame[4] = payload & 0xFF;
  frame[5] = (payload >> 8) & 0xFF;
}
static Bytes ack(uint8_t command, size_t size) {
  Bytes frame(size, 0);
  const Bytes header{0xFD, 0xFC, 0xFB, 0xFA}, footer{4, 3, 2, 1};
  std::copy(header.begin(), header.end(), frame.begin());
  std::copy(footer.begin(), footer.end(), frame.end() - 4);
  frame[6] = command;
  frame[7] = 1;
  length(frame);
  return frame;
}
static Bytes params() {
  auto frame = ack(0x61, 38);
  frame[10] = 0xAA;
  frame[11] = 8;
  frame[12] = 7;
  frame[13] = 6;
  for (size_t g = 0; g < 9; ++g) {
    frame[14 + g] = 10 + g;
    frame[23 + g] = 30 + g;
  }
  frame[32] = 0x34;
  frame[33] = 0x12;
  return frame;
}
static Config parameters(const Config &initial) {
  auto expected = initial;
  expected.max_move_gate = 7;
  expected.max_still_gate = 6;
  expected.timeout_seconds = 0x1234;
  for (size_t g = 0; g < 9; ++g) {
    expected.gate_move_threshold[g] = 10 + g;
    expected.gate_still_threshold[g] = 30 + g;
  }
  return expected;
}
static Bytes join(Bytes first, const Bytes &second) {
  first.insert(first.end(), second.begin(), second.end());
  return first;
}

using Test = std::pair<std::string, std::function<void()>>;
static std::vector<Test> tests() {
  std::vector<Test> cases;
  auto reject = [&](std::string name, Bytes frame, bool failed_ack = false) {
    cases.emplace_back(std::move(name), [frame, failed_ack]() {
      Fixture f;
      f.rejected(frame, failed_ack);
    });
  };
  cases.emplace_back("parameters_normal38", [] {
    Fixture f;
    f.parse(params());
    f.accepted(parameters(f.initial));
  });
  const std::vector<std::pair<std::string, Bytes>> valid{{"parameters", params()},        {"resolution", ack(0xAB, 16)},
                                                         {"mac", ack(0xA5, 20)},          {"version", ack(0xA0, 22)},
                                                         {"enter_config", ack(0xFF, 18)}, {"generic", ack(0x62, 14)}};
  for (const auto &entry : valid) {
    const auto &frame = entry.second;
    for (size_t n = 0; n < frame.size(); ++n)
      reject(entry.first + "_prefix_" + std::to_string(n), Bytes(frame.begin(), frame.begin() + n));
    for (size_t n = 10; n < frame.size(); ++n) {
      Bytes shortened(frame.begin(), frame.begin() + n - 4);
      shortened.insert(shortened.end(), frame.end() - 4, frame.end());
      length(shortened);
      reject(entry.first + "_short_complete_" + std::to_string(n), shortened);
    }
    for (size_t offset : {size_t(0), size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(7), size_t(8),
                          size_t(9), frame.size() - 4, frame.size() - 3, frame.size() - 2, frame.size() - 1}) {
      auto bad = frame;
      bad[offset] ^= 0x80;
      reject(entry.first + "_bad_byte_" + std::to_string(offset), bad, offset == 8 || offset == 9);
    }
    for (size_t offset : {size_t(8), size_t(9)}) {
      auto bad = frame;
      bad[offset] = 1;
      reject(entry.first + "_status_" + std::to_string(offset), bad, true);
    }
    auto bad = frame;
    bad[7] = 0;
    reject(entry.first + "_command_high_zero", bad);
  }
  for (auto change : std::vector<std::pair<size_t, uint8_t>>{{10, 0}, {11, 9}, {11, 6}, {12, 9}, {13, 9}}) {
    auto bad = params();
    bad[change.first] = change.second;
    reject("parameters_invalid_" + std::to_string(change.first) + "_" + std::to_string(change.second), bad);
  }
  for (size_t offset = 14; offset <= 31; ++offset) {
    auto bad = params();
    bad[offset] = 101;
    reject("parameters_threshold_" + std::to_string(offset), bad);
  }
  for (size_t gate : {size_t(12), size_t(13)}) {
    auto bad = params();
    bad[11] = 5;
    bad[12] = 5;
    bad[13] = 5;
    bad[gate] = 6;
    reject("parameters_gate_exceeds_supported_" + std::to_string(gate), bad);
  }
  cases.emplace_back("parameters_valid_boundaries", [] {
    Fixture f;
    auto frame = params();
    frame[12] = 0;
    frame[13] = 8;
    frame[14] = 0;
    frame[31] = 100;
    frame[32] = 0xFF;
    frame[33] = 0xFF;
    auto expected = parameters(f.initial);
    expected.max_move_gate = 0;
    expected.max_still_gate = 8;
    expected.gate_move_threshold[0] = 0;
    expected.gate_still_threshold[8] = 100;
    expected.timeout_seconds = 65535;
    f.parse(frame);
    f.accepted(expected);
  });
  for (uint16_t value : {uint16_t(2), uint16_t(256), uint16_t(65535)}) {
    auto bad = ack(0xAB, 16);
    bad[10] = value & 0xFF;
    bad[11] = value >> 8;
    reject("resolution_invalid_" + std::to_string(value), bad);
  }
  for (uint8_t value : {uint8_t(0), uint8_t(1)}) {
    cases.emplace_back("resolution_valid_" + std::to_string(value), [value] {
      Fixture f;
      auto frame = ack(0xAB, 16);
      frame[10] = value;
      f.parse(frame);
      auto expected = f.initial;
      expected.distance_resolution = value;
      f.accepted(expected);
    });
  }
  for (bool enabled : {false, true}) {
    cases.emplace_back(enabled ? "mac_actual20" : "mac_disabled20", [enabled] {
      Fixture f;
      auto frame = ack(0xA5, 20);
      const Bytes mac = enabled ? Bytes{0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC} : Bytes{8, 5, 4, 3, 2, 1};
      std::copy(mac.begin(), mac.end(), frame.begin() + 10);
      f.parse(frame);
      auto expected = f.initial;
      expected.bluetooth_enabled = enabled;
      f.accepted(expected);
    });
  }
  cases.emplace_back("version_valid22", [] {
    Fixture f;
    auto frame = ack(0xA0, 22);
    frame[12] = 0x23;
    frame[13] = 4;
    frame[14] = 0x12;
    frame[15] = 0x34;
    frame[16] = 0x56;
    frame[17] = 0x78;
    f.parse(frame);
    f.accepted(f.initial, 0);
    assert(f.version.publishes == 1 && f.version.state == "V4.23.78563412");
  });
  cases.emplace_back("generic_valid14", [] {
    Fixture f;
    f.parse(ack(0x62, 14));
    f.accepted(f.initial, 0);
  });
  cases.emplace_back("enter_config_valid18", [] {
    Fixture f;
    auto frame = ack(0xFF, 18);
    frame[10] = 1;
    frame[12] = 64;
    f.parse(frame);
    f.accepted(f.initial, 0);
  });
  for (uint8_t command : {uint8_t(0x61), uint8_t(0xAB), uint8_t(0xA5), uint8_t(0xA0), uint8_t(0xFF), uint8_t(0xA4)}) {
    for (size_t status_offset : {size_t(8), size_t(9)}) {
      auto frame = ack(command, 14);
      frame[status_offset] = 1;
      cases.emplace_back("failed_ack_progress_" + std::to_string(command) + "_" + std::to_string(status_offset),
                         [frame] {
                           Fixture f;
                           f.handler.disable_engineering_mode();
                           f.rejected(frame, true);
                           f.handler.loop();
                           assert(f.uart.writes.size() == 1 && f.uart.writes[0][6] == 0xFF);
                           assert(fake_now == 100);
                         });
    }
  }
  reject("oversized_ack257", ack(0x62, 257));
  cases.emplace_back("null_ack", [] {
    Fixture f;
    f.handler.handle_ack_frame_(nullptr, 38);
    assert(equal(f.handler.get_backend_config(), f.initial));
    assert(preference_state.saves == 0 && preference_state.syncs == 0);
    assert(f.handler.waiting());
  });
  cases.emplace_back("bluetooth_ack_clock", [] {
    Fixture f;
    f.parse(ack(0xA4, 14));
    f.accepted(f.initial, 0);
    assert(f.handler.readback_pending());
    fake_now += 1499;
    f.handler.loop();
    assert(f.uart.writes.empty());
    fake_now += 1;
    f.handler.loop();
    assert(!f.handler.readback_pending());
    f.handler.loop();
    assert(f.uart.writes.size() == 1);
  });
  for (size_t split = 1; split < 38; ++split) {
    cases.emplace_back("stream_fragment_" + std::to_string(split), [split] {
      Fixture f;
      const auto frame = params();
      f.feed(Bytes(frame.begin(), frame.begin() + split));
      assert(equal(f.handler.get_backend_config(), f.initial));
      assert(preference_state.saves == 0 && preference_state.syncs == 0 && f.handler.waiting());
      f.feed(Bytes(frame.begin() + split, frame.end()));
      f.accepted(parameters(f.initial));
    });
  }
  for (size_t suffix = 1; suffix <= 3; ++suffix) {
    cases.emplace_back("stream_bytewise_noise_partial_header_" + std::to_string(suffix), [suffix] {
      Fixture f;
      const auto frame = params();
      f.feed(join({0, 0xFD, 0xFC, 0, 0xF4, 0xF3, 0}, Bytes(frame.begin(), frame.begin() + suffix)));
      for (size_t i = suffix; i < frame.size(); ++i)
        f.feed({frame[i]});
      f.accepted(parameters(f.initial));
    });
  }
  for (const std::string kind : {"bad_footer", "oversize", "swallow_header", "short_length"}) {
    cases.emplace_back("stream_recover_" + kind, [kind] {
      Fixture f;
      Bytes bad;
      if (kind == "bad_footer") {
        bad = params();
        bad.back() = 0;
      }
      if (kind == "oversize") {
        bad = {0xFD, 0xFC, 0xFB, 0xFA, 0xFF, 0xFF, 0x61, 1};
      }
      if (kind == "short_length") {
        bad = {0xFD, 0xFC, 0xFB, 0xFA, 0, 0, 0x61, 1};
      }
      if (kind == "swallow_header") {
        bad = ack(0x61, 14);
        // Candidate ends inside the next reply, so recovery must reevaluate already buffered bytes.
        bad[4] = 16;
      }
      f.feed(join(bad, params()));
      f.accepted(parameters(f.initial));
    });
  }
  cases.emplace_back("stream_recover_data_bad_footer", [] {
    Fixture f;
    Bytes bad(21, 0);
    bad[0] = 0xF4;
    bad[1] = 0xF3;
    bad[2] = 0xF2;
    bad[3] = 0xF1;
    bad[4] = 19;
    f.feed(join(bad, params()));
    f.accepted(parameters(f.initial));
  });
  cases.emplace_back("stream_failed_then_valid", [] {
    Fixture f;
    auto bad = params();
    bad[9] = 1;
    f.feed(bad);
    assert(equal(f.handler.get_backend_config(), f.initial));
    assert(preference_state.saves == 0 && preference_state.syncs == 0 && !f.handler.waiting());
    f.feed(params());
    f.accepted(parameters(f.initial));
  });
  cases.emplace_back("stream_consecutive", [] {
    Fixture f;
    auto resolution = ack(0xAB, 16);
    f.feed(join(params(), resolution));
    auto expected = parameters(f.initial);
    expected.distance_resolution = 0;
    f.accepted(expected, 2);
  });
  cases.emplace_back("stream_budget128", [] {
    Fixture f;
    Bytes noise(127, 0);
    f.uart.inject(join(noise, params()));
    f.handler.loop();
    assert(f.uart.reads == 128 && f.uart.available() == 37);
    assert(equal(f.handler.get_backend_config(), f.initial));
    assert(preference_state.saves == 0 && preference_state.syncs == 0 && f.handler.waiting());
    f.handler.loop();
    assert(f.uart.available() == 0);
    f.accepted(parameters(f.initial));
  });
  cases.emplace_back("stream_max_buffer256", [] {
    Fixture f;
    f.uart.inject(ack(0x62, 256));
    f.handler.loop();
    assert(f.uart.available() == 128 && f.handler.waiting());
    f.handler.loop();
    assert(f.uart.available() == 0);
    f.accepted(f.initial, 0);
    f.feed(params());
    f.accepted(parameters(f.initial));
  });
  return cases;
}

int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  const auto cases = tests();
  if (std::string(argv[1]) == "--list") {
    for (const auto &test : cases)
      std::cout << test.first << '\n';
    return 0;
  }
  for (const auto &test : cases) {
    if (test.first == argv[1]) {
      test.second();
      std::cout << "PASS " << test.first << '\n';
      return 0;
    }
  }
  return 2;
}
