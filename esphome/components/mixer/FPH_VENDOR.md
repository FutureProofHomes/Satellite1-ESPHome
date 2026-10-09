# Vendored mixer — FutureProofHomes

This directory is a **byte-identical copy of upstream ESPHome's `mixer` component**, pinned to the
exact version in `requirements.txt` (currently **2026.9.1**), **except for one addition**, marked
with `// FPH:`.

## The one diff

A per-input gain on `SourceSpeaker`:

- `speaker/mixer_speaker.h`: `set_gain(float linear)` and `get_gain()`, plus two atomic Q31 members
  (`gain_target_q31_`, `gain_current_q31_`) and the `<cstdint>` include.
- `speaker/mixer_speaker.cpp`: the `<gain.h>` include, `GAIN_RAMP_STEPS`, the gain pass in
  `process_data_from_source()` right after the existing ducking call, the `set_gain()`/`get_gain()`
  bodies, and one line in `start_()` that starts a new stream at the requested gain.

`audio_gain_reconcile` in `config/common/voice_assistant.yaml` uses it to give the media and
announcement inputs their own levels under one amplifier level. Upstream's only per-input control is
`apply_ducking()`, which stops at `esp_audio_libs::ducking::MAX_DB_REDUCTION` (50 dB), so a music
volume of 0 under a louder announcement level could never be silent. `esp_audio_libs::gain::apply()`
with a scale of 0 is exact silence. A change ramps in `GAIN_RAMP_STEPS` equal steps across the next
block the mixer reads (at most 50 ms), so a step does not click. Unity skips the pass entirely.

Ducking is untouched and still works; the firmware simply no longer calls it.

## Re-syncing on an ESPHome bump

1. Bump `requirements.txt`, rebuild `.venv` (`scripts/setup_build_env.sh`).
2. Re-copy the component:
   `cp -R .venv/lib/python*/site-packages/esphome/components/mixer/ esphome/components/mixer/`
   (then delete `__pycache__`; keep this file).
3. Re-apply the `// FPH:` blocks — `git diff` against the pre-copy state shows exactly what to
   restore; they are the only diff there should be.
4. Re-verify that `process_data_from_source()` is still the one place each block of a source's audio
   is exposed to the mixer exactly once (the early return on `available() > 0` is what stops a
   partially consumed block being scaled twice).

## Exit plan

PR a per-source gain upstream to esphome/esphome. Once released, delete this directory and the
`mixer` entries in both `external_components` lists (`config/satellite1.yaml` and
`config/common/components.external.yaml`), and point the reconciler at the upstream call.
