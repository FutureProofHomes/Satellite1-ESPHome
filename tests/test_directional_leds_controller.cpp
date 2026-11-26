#include "esphome/components/satellite1_directional_leds/directional_leds.h"
#include <cassert>
#include <iostream>

using esphome::satellite1_directional_leds::DirectionalLEDs;

static void poll(DirectionalLEDs &leds, uint32_t now) {
  esphome::test_millis = now;
  leds.loop();
}

static void session_transition_test() {
  esphome::test_millis = 0;
  esphome::satellite1::Satellite1 satellite;
  esphome::i2s_audio::I2SAudioSpeaker speaker;
  DirectionalLEDs leds;
  leds.set_satellite(&satellite);
  leds.set_speaker(&speaker);
  unsigned changes = 0;
  leds.add_on_availability_callback([&]() { ++changes; });
  leds.set_enabled(true);
  leds.set_phase(true, false);
  poll(leds, 0);
  satellite.frame = 2;
  poll(leds, 50);
  assert(leds.available() && changes == 1);
  leds.set_phase(false, true);
  assert(leds.can_reply() && speaker.metering);

  // An idle gap must not force the original effect at the next wake word or
  // recursively execute control_leds from its own set_phase() call.
  esphome::test_millis = 5000;
  leds.set_phase(true, false);
  assert(leds.available() && changes == 1);
  assert(!leds.has_listening_direction() && !leds.can_reply() && !speaker.metering);
  satellite.responding = false;
  poll(leds, 5000);
  assert(leds.available() && changes == 1);
  satellite.responding = true;
  satellite.angle = 18;
  satellite.frame = 3;
  poll(leds, 5050);
  assert(leds.has_listening_direction() && leds.listening_position() == 18);
  leds.set_phase(false, true);
  assert(leds.can_reply() && leds.reply_position() == 18);  // New session, new votes.

  // The wake-word grace period cannot conceal a stalled DSP indefinitely.
  esphome::test_millis = 10000;
  leds.set_phase(true, false);
  poll(leds, 10000);
  assert(leds.available());
  poll(leds, 10501);
  assert(!leds.available() && changes == 2);
}

static void transient_read_test() {
  esphome::test_millis = 0;
  esphome::satellite1::Satellite1 satellite;
  esphome::i2s_audio::I2SAudioSpeaker speaker;
  DirectionalLEDs leds;
  leds.set_satellite(&satellite);
  leds.set_speaker(&speaker);
  leds.set_enabled(true);
  leds.set_phase(true, false);
  poll(leds, 0);
  satellite.frame = 2;
  poll(leds, 50);

  satellite.responding = false;
  poll(leds, 100);
  assert(leds.available());  // One busy/missed SPI read must not change effects.
  satellite.responding = true;
  satellite.frame = 3;
  poll(leds, 150);
  assert(leds.available() && satellite.transfers == 4);  // Retry in 50 ms, not a second.

  satellite.responding = false;
  poll(leds, 650);
  assert(leds.available());
  poll(leds, 700);
  assert(!leds.available());  // Sustained transport loss still falls back after 500 ms.
  satellite.responding = true;
  satellite.frame = 4;
  poll(leds, 750);
  assert(leds.available());

  // Invalid telemetry receives the same bounded treatment as a missed transfer.
  satellite.angle = 24;
  poll(leds, 800);
  assert(leds.available());
  poll(leds, 1300);
  assert(!leds.available());
  satellite.angle = 6;
  satellite.frame = 5;
  poll(leds, 1350);
  assert(leds.available());
}

int main() {
  session_transition_test();
  transient_read_test();
  esphome::test_millis = 0;
  esphome::satellite1::Satellite1 satellite;
  esphome::i2s_audio::I2SAudioSpeaker speaker;
  DirectionalLEDs leds;
  leds.set_satellite(&satellite);
  leds.set_speaker(&speaker);
  unsigned changes = 0;
  leds.add_on_availability_callback([&]() { ++changes; });

  leds.set_phase(true, false);
  poll(leds, 0);
  assert(satellite.transfers == 0 && !speaker.metering && !leds.available());
  leds.set_enabled(true);
  poll(leds, 0);
  assert(!leds.available());
  satellite.frame = 2;
  poll(leds, 50);
  assert(leds.available() && leds.has_listening_direction() && changes == 1);
  assert(!speaker.metering);

  leds.set_phase(false, false);
  leds.set_phase(false, true);
  assert(leds.can_reply() && leds.reply_position() == 6 && speaker.metering);
  satellite.angle = 18;
  satellite.frame = 99;
  poll(leds, 1000);
  assert(leds.reply_position() == 6 && satellite.transfers == 2);

  leds.set_enabled(false);
  assert(!leds.available() && !leds.can_reply() && !speaker.metering);
  poll(leds, 2000);
  assert(satellite.transfers == 2);  // Off performs no telemetry work.
  leds.set_enabled(true);
  leds.set_phase(true, false);
  poll(leds, 2000);
  satellite.frame = 100;
  poll(leds, 2050);
  assert(leds.has_listening_direction());
  poll(leds, 2600);
  assert(!leds.available());  // Frozen DSP switches the renderer to the standard effect.
  satellite.frame = 101;
  poll(leds, 2650);
  assert(leds.available());

  satellite.ready = false;
  poll(leds, 2700);
  assert(!leds.available() && !speaker.metering);
  satellite.ready = true;
  poll(leds, 2800);
  assert(!leds.available());
  satellite.frame = 102;
  poll(leds, 2850);
  assert(leds.available());

  satellite.mute.muted = true;
  const auto before_mute = satellite.transfers;
  poll(leds, 2900);
  assert(!leds.available() && !speaker.metering && satellite.transfers == before_mute);
  satellite.mute.muted = false;
  poll(leds, 2910);
  ++satellite.frame;
  poll(leds, 2960);
  assert(leds.available());

  leds.set_enabled(false);
  leds.set_enabled(true);
  satellite.responding = false;
  const unsigned before = satellite.transfers;
  poll(leds, 3000);
  poll(leds, 4000);
  poll(leds, 5000);
  poll(leds, 60000);
  assert(satellite.transfers == before + 3 && !leds.available());
  leds.set_enabled(false);
  leds.set_enabled(true);
  poll(leds, 61000);
  assert(satellite.transfers == before + 4);  // A live off/on permits a new capability probe.

  std::cout << "Live toggle, fallback, bounded probing, restart and reply freeze checks passed.\n";
}
