# Add optional directional voice LEDs and capability-gated XMOS hints

This PR adds a live **Directional voice LEDs** Home Assistant switch:

- **Off:** FPH’s existing voice animations.
- **On:** a direction indicator while listening, followed by a reply animation at the remembered speaker position.

The new effects use a blue body with a white direction tip. The switch defaults to off and restores the user’s choice across restarts.

### What’s included

- Smoothed listening direction and reply pulsing based on playback level.
- Direction availability preserved across conversations and brief missed telemetry reads.
- Standard-animation fallback when direction telemetry is missing, unsupported or stalled.
- Existing thinking/status effects and mute, volume, timer and other LED priorities preserved.
- Playback/listening hints that operate independently of the LED switch.

The LED renderer reads direction telemetry and playback levels without changing audio samples. A separate component sends the host hints.

### XMOS integration

Companion: [XMOS PR #133](https://github.com/FutureProofHomes/Satellite1-XMOS/pull/133).

Direction animations require its SPI resource 233 telemetry. They can work with the existing audio pipeline or the optional beamforming pipelines.

Host hints are **enabled by default** through `xmos_audio_hints: "true"`. They are sent only after XMOS explicitly advertises support. Unsupported images receive no hint writes, and supporting XMOS firmware expires stale hints automatically.

Setting `xmos_audio_hints: "false"` disables hint probes and writes.

Wake detection, speech timeouts, ducking, volume and audio samples are unchanged. The configured XMOS image, version and download URLs are also preserved.

### Paired test helper

After building and packaging `speech_tracking` in the XMOS checkout:

```sh
.venv/bin/python scripts/prepare_speech_tracking_test.py \
  --xmos-artifacts /path/to/Satellite1-XMOS/build_test/artifacts

.venv/bin/python -m esphome run \
  config/satellite1.speech-tracking-test.yaml --no-logs
```

The helper verifies the XMOS artifacts and generates a test configuration with the matching image, version and checksum. ESPHome installs that image automatically when needed. The standard configuration’s XMOS image selection is unchanged.
