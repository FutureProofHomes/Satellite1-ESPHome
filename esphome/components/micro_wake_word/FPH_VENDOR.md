# Vendored micro_wake_word — FutureProofHomes

This directory is a **byte-identical copy of upstream ESPHome's `micro_wake_word` component**,
pinned to the exact version in `requirements.txt` (currently **2026.8.1**), **except for one
addition**: the high-water register, every line of which is marked with `// FPH:`.

## The one diff

- `streaming_model.h`: `#include <atomic>`, `WakeWordModel::take_high_water()`, and the
  `fph_high_water_` member (all `// FPH:`-marked).
- `streaming_model.cpp`: two lines in `WakeWordModel::determine_detected()` keeping the max
  sliding-window mean in the register (marked).

That register is what makes sub-cutoff scores observable at all — it feeds the web UI's live
room-pressure margin bars, the scored close-call record, and the Wake Word Tuner's max-mean
measurements (see `mww_runtime_loader`). Nothing else is changed, deliberately: the vendor exists
for these ~10 lines only (owner decision, September 2026 — keep maintenance minimal).

## Re-syncing on an ESPHome bump

1. Bump `requirements.txt`, rebuild `.venv` (`scripts/setup_build_env.sh`).
2. Re-copy the component:
   `cp -R .venv/lib/python*/site-packages/esphome/components/micro_wake_word/ esphome/components/micro_wake_word/`
   (then delete `__pycache__`; keep this file).
3. Re-apply the `// FPH:` block — `git diff` against the pre-copy state shows exactly what to
   restore; it is the only diff there should be. Every restored line must carry an `FPH` marker
   (comment continuations included), because the verifier below checks line by line.
4. Run `scripts/check_vendored_mww.sh` (CI runs it on every PR — the `vendored micro_wake_word`
   job in `lint.yaml`). It fails the build unless the copy is upstream plus FPH-marked lines
   only, and it re-checks the two detection log lines `mww_runtime_loader::on_mww_log_` parses
   (that dependency predates the vendor and is documented there).

## Exit plan

PR the register upstream to esphome/esphome. Once released, delete this directory and the
`micro_wake_word` entries in both `external_components` lists, and the loader keeps working
against upstream unchanged.
