# Optional directional voice LEDs

The **Directional voice LEDs** Home Assistant switch changes both voice
animations live, without a reboot. It defaults to off and restores the user's
choice after a restart.

- **Off:** the existing FPH listening and replying animations.
- **On:** a smoothed direction indicator while listening, followed by a reply
  effect that expands/contracts with playback level at the remembered speaker
  direction. Both use a blue body with a white tip at the indicated direction;
  the reply tip also pulses with the smoothed audio envelope. Without a valid
  listening direction, the neutral display stays blue with no white pointer.
  Thinking and other existing status effects are retained.

Direction requires a compatible XMOS image with the four-microphone direction
servicer enabled. This is the dependency supplied by the companion
[XMOS PR #133](https://github.com/FutureProofHomes/Satellite1-XMOS/pull/133).
Merging XMOS source alone does not enable the feature in a release: its image
must be built with `ENABLE_SAT1_DIRECTION_FINDING=ON` (or use the optional
`calibrated_beam` or `speech_tracking` target, which enables it). Direction diagnostics can also run
with FPH's normal audio pipeline; the LEDs do not require beamforming or IC.

Recommended rollout: merge [XMOS PR #133](https://github.com/FutureProofHomes/Satellite1-XMOS/pull/133), publish a direction-enabled XMOS
image, then configure ESPHome to embed that image with its matching version
and checksum. This ESPHome change leaves the currently configured XMOS image
version and download URLs unchanged. Until a compatible image is installed,
the standard animations remain available through the fallback below.

Without compatible telemetry, the switch can remain on but the standard FPH
effects are used. Unsupported firmware is probed only three times per
connection/enable; toggling off and on permits a new probe. Freshness depends
on an advancing XMOS frame counter, not the reported firmware version. A stale
stream falls back after 500 ms without frame progress; a disconnected or muted
stream clears availability immediately. A missed SPI read is retried after
50 ms on a previously supported image, without changing effects while recent
progress is still valid. Starting a new listening session preserves that
availability, allows up to 500 ms to resume telemetry after the idle polling
pause, and clears only the previous speaker evidence. Weak or briefly
missing direction estimates produce a neutral listening display after an
800 ms hold. A new listening session clears the previous reply direction.
If the switch is enabled during a reply, directional replying starts after the
next listening session supplies a reliable position.

The reply position is the confidence-weighted dominant listening direction,
so a short late sound does not redirect it. It is frozen during thinking and
playback to avoid following the device's own speaker. Playback reactivity is
measured read-only from PCM accepted by the ESP32 I2S driver, after software
volume and before hardware DAC volume. The renderer and meter do not alter
audio samples; the separate capability-gated context bridge is described below. The LED component requests metering only when a directional reply can be used.
An independent audio-hint subscription can also keep metering active; disabling
either consumer does not disable the other.

The existing LED priority rules for flashing, provisioning, connection state,
buttons, volume, warnings, timers and mute remain in charge. Microphone mute
also suspends direction polling and clears the remembered position.

## Capability-gated playback/listening hints

The new `satellite1_audio_hints` component observes the existing voice-assistant
phase and read-only PCM level. The shared ESPHome configuration enables it by
default with `xmos_audio_hints: "true"`. A supporting XMOS image receives hints
automatically; an unsupported image receives only bounded capability probes and
no hint writes. Set `xmos_audio_hints: "false"` in the build's substitutions to
disable all hint probes and writes. The existing XMOS version, download URLs,
wake/voice actions, speech timeouts, playback rules and LED-switch default are
unchanged.

The companion optional XMOS `speech_tracking` profile advertises support through
resource 233 command 0x83: signature `S1D`, protocol version 1, then a uint32
capability word with bit 0 set. Only after verifying that response does the
bridge send SET_HOST_HINTS (command 4) bit 0 (playback) and bit 3 (confirmed listening). Old
images may reject the probe or advertise zero support; they get no writes.
Capability probing is non-retrying and limited to three attempts per audio
availability epoch. Context updates are polled every 125 ms, with a heartbeat
once per second, independent of the LED switch. The supporting DSP expires stale
hints after 160 advancing frames without a refresh.

To prepare a local paired test without editing production image selection:

```sh
.venv/bin/python scripts/prepare_speech_tracking_test.py \
  --xmos-artifacts /path/to/Satellite1-XMOS/build_test/artifacts
.venv/bin/python -m esphome compile config/satellite1.speech-tracking-test.yaml
```

The helper verifies the artifact manifest, image size, SHA-256 and MD5, then
writes only `config/satellite1.speech-tracking-test.yaml` (ignored by Git).
It selects the verified local XMOS image/version and sets hints true. FPH's
automatic image matching otherwise retains its original behavior; manually
installing a new DSP while still selecting an old embedded image can cause the
old image to be installed again.

The updated local pair needs its own hardware validation for quiet/music
commands, immediate direction at command onset, reply animation, LED toggle,
mute/unmute, playback interruptions and cold boots. Earlier dev.4 physical
observations below do not qualify the new profile. The current native Tater r4
device was not flashed or changed while preparing these PR updates.

## Validation

Run `bash scripts/test_directional_leds.sh` for direction wrap, confidence,
stale/frozen data, direction memory, session reset, and 8/16/24/32-bit stereo
PCM checks under address/undefined-behavior sanitizers. The actual controller
is exercised with transport/speaker test doubles for live toggling, limited
probing, mute, reconnect, fallback and freezing direction during replies.
Regression checks also cover consecutive conversations after an idle gap,
missed/invalid SPI reads, prompt recovery, and sustained transport/DSP stalls.
The additional audio-hint suite covers explicit opt-out zero traffic, capability
gating, mute, heartbeat, failed-write recovery, reconnect, clock wrap and
independent LED/hint meter ownership. Run
`python -m unittest discover -s tests -p 'test_speech_tracking_package.py'` for
package integrity and production-file protection. Validate/compile both the
normal config and generated opt-in config with the pinned ESPHome requirements.

On November 24, 2026, the original two host sanitizer suites, configuration validation,
the local bench firmware build and [CI host/firmware jobs](https://github.com/TaterTotterson/Satellite1-ESPHome/actions/runs/45462349479)
passed at source commit `303f498451e90a06974922a9e7cfc89eef952c82`.
The bench image was flashed and verified on a Sat1 with XMOS `1.2.0-dev.4`
(protocol `0x11`). Wi-Fi and Home Assistant pairing were retained, and the
device reconnected successfully. Factory image SHA256:
`11abaf964cd85ed286e0d6035134bc5eb3d23c8660f1cb9ddfe6297daa94afd6`.

With the switch enabled, the tester confirmed correct listening direction
from two opposite sides and reply pulsing in the remembered direction.
The tester also confirmed that switching off immediately restored FPH's
original effects and switching on restored the new effects, with clear voice
replies in both modes and no reboot.
These are user-reported physical observations, not quantitative direction or
audio-quality measurements.

Further physical checks: orientation from the remaining sides, persistence across restart, fallback
with a stock XMOS image, and priority of mute/volume/timer effects.

The blue/white-tip styling revision at `f4d4d5b` was also compiled, flashed and
verified on November 24. It booted `tater-directional-leds-white-tip-dev`,
retained XMOS dev.4 and reconnected to Wi-Fi/Home Assistant. The tester
confirmed that the white tips and surrounding blue animation looked right
during both listening and replying. Factory image SHA256:
`cc76750566702f2cbfdd3caa9a6dab23fb2b006d5419035ba16961a66977325a`.

Later testing exposed brief returns to the original rotating effect. The
controller cleared telemetry availability on every new listening session and
immediately fell back for one missed SPI read, delaying the next poll by a
second. A regression test reproduced the session-start failure before the fix.
The controller now keeps telemetry liveness separate from per-conversation
direction evidence and tolerates brief read failures within the 500 ms progress
window. A supported image is retried at the normal 50 ms interval; unsupported
images retain their limited capability probes.

Both sanitizer suites and the local ESPHome firmware build passed after this
correction. The `tater-directional-leds-stability-dev` host was installed over
OTA on the same Sat1, retaining XMOS `1.2.0-dev.4` and the enabled switch. We
confirmed that the new animations stayed on through three consecutive voice
requests. Logs show three completed replies, no `control_leds` re-entry warnings
and no telemetry-availability fallback events during those requests. OTA image
SHA256: `d3fe0527cea90bda4d992c9a45509a008086290e85e63ec31a4aa19d44f6efc2`.
