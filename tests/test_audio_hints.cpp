#include "esphome/components/satellite1_audio_hints/audio_hints.h"
#include "esphome/components/satellite1_directional_leds/directional_leds.h"
#include <cassert>
#include <iostream>

using esphome::satellite1_audio_hints::AudioHints;
using esphome::satellite1_directional_leds::DirectionalLEDs;

static void poll(AudioHints &hints, uint32_t now) {
  esphome::test_millis = now;
  hints.loop();
}

static void unsupported_images_are_read_only() {
  for (unsigned variant = 0; variant < 3; ++variant) {
    esphome::satellite1::Satellite1 satellite;
    esphome::i2s_audio::I2SAudioSpeaker speaker;
    satellite.capability_present = variant != 0;
    satellite.hints_capability = false;
    satellite.valid_capability = variant != 2;
    AudioHints hints;
    hints.set_satellite(&satellite);
    hints.set_speaker(&speaker);
    hints.set_voice_active([]() { return true; });
    hints.set_enabled(true);
    for (unsigned now = 0; now < 10000; now += 125)
      poll(hints, now);
    assert(!hints.supported() && !speaker.metering);
    assert(satellite.write_attempts == 0);
    assert(satellite.capability_reads == (variant == 1 ? 1 : 3));
  }
}

static void context_heartbeat_mute_and_failure() {
  esphome::satellite1::Satellite1 satellite;
  esphome::i2s_audio::I2SAudioSpeaker speaker;
  satellite.capability_present = true;
  AudioHints hints;
  bool voice = false;
  hints.set_satellite(&satellite);
  hints.set_speaker(&speaker);
  hints.set_voice_active([&]() { return voice; });
  hints.set_enabled(true);
  poll(hints, 0);
  assert(hints.supported() && speaker.metering && satellite.write_attempts == 0);
  poll(hints, 125);
  assert(satellite.hint_writes.back() == 1);
  poll(hints, 250);
  assert(satellite.write_attempts == 1);
  voice = true;
  poll(hints, 375);
  assert(satellite.hint_writes.back() == 9);
  speaker.level = 0;
  poll(hints, 500);
  assert(satellite.hint_writes.back() == 8);
  satellite.mute.muted = true;
  poll(hints, 625);
  assert(satellite.hint_writes.back() == 0);
  const unsigned writes = satellite.write_attempts;
  poll(hints, 1500);
  assert(satellite.write_attempts == writes);
  poll(hints, 1625);
  assert(satellite.write_attempts == writes + 1 && satellite.hint_writes.back() == 0);
  satellite.write_ok = false;
  speaker.level = 0.2f;
  poll(hints, 1750);
  assert(satellite.write_attempts == writes + 2);
  for (unsigned now = 1751; now < 1875; ++now)
    poll(hints, now);
  assert(satellite.write_attempts == writes + 2);
  satellite.write_ok = true;
  poll(hints, 1875);
  assert(satellite.hint_writes.back() == 1);
  satellite.ready = false;
  poll(hints, 2000);
  assert(!hints.supported() && !speaker.metering);
  satellite.capability_present = false;
  satellite.ready = true;
  poll(hints, 2125);
  assert(!hints.supported() && !speaker.metering);
}

static void independent_led_metering_and_wrap() {
  esphome::satellite1::Satellite1 satellite;
  esphome::i2s_audio::I2SAudioSpeaker speaker;
  satellite.capability_present = true;
  AudioHints hints;
  hints.set_satellite(&satellite);
  hints.set_speaker(&speaker);
  hints.set_voice_active([]() { return true; });
  hints.set_enabled(true);
  DirectionalLEDs leds;
  leds.set_satellite(&satellite);
  leds.set_speaker(&speaker);
  poll(hints, UINT32_MAX - 100);
  assert(hints.supported());
  leds.set_enabled(true);
  leds.set_enabled(false);
  assert(speaker.metering);  // Disabling LEDs must not disable the audio hint meter.
  poll(hints, 24);
  assert(satellite.hint_writes.back() == 9);
  speaker.set_level_meter_enabled(true);
  satellite.ready = false;
  poll(hints, 25);
  assert(speaker.metering);  // Hint teardown must not disable the independent LED user.
  speaker.set_level_meter_enabled(false);
  assert(!speaker.metering);
}

int main() {
  {
    esphome::satellite1::Satellite1 satellite;
    esphome::i2s_audio::I2SAudioSpeaker speaker;
    AudioHints hints;
    hints.set_satellite(&satellite);
    hints.set_speaker(&speaker);
    hints.set_enabled(false);
    for (unsigned now = 0; now < 5000; now += 125) poll(hints, now);
    assert(satellite.transfers == 0 && !speaker.metering);
  }
  unsupported_images_are_read_only();
  context_heartbeat_mute_and_failure();
  independent_led_metering_and_wrap();
  std::cout << "Audio hint capabilities, heartbeat, mute, recovery, clock wrap and LED independence passed.\n";
}
