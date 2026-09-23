# Vendored voice_assistant — FutureProofHomes

This directory is a **byte-identical copy of upstream ESPHome's `voice_assistant` component**,
pinned to the exact version in `requirements.txt` (currently **2026.8.1**), **except for one
addition**, marked with `// FPH:`.

## The one diff

- `voice_assistant.h`: a public getter, `get_continue_conversation()`, over the existing
  protected `continue_conversation_` member (plus its `// FPH:` comment block).

That flag is Home Assistant's authoritative "a listen window follows this answer" verdict: the
component sets it from the `continue_conversation` argument of `VOICE_ASSISTANT_INTENT_END`
(`voice_assistant.cpp`, `on_event`) and from `msg.start_conversation` on an announce
(`on_announce`), both strictly before `on_end` fires, and consumes it only afterwards (the
`STREAMING_RESPONSE` exit) — so it is valid when an `on_end` automation reads it. The remote sync
guard in `config/common/tts_routing.yaml` gates its silence padding on it, so a routed one-shot
answer no longer pays 0.5–2 s of closed mic and held ducking for a listen window that was never
going to open. Nothing else is changed, deliberately: the vendor exists for this one getter
(matching the `micro_wake_word` vendor's keep-maintenance-minimal rule).

A trailing-"?" text heuristic was considered and rejected in planning: LLM agents continue
conversations without a trailing question mark, a question mark does not guarantee continuation,
and the component truncates TTS text over 500 characters before any trigger sees it — the
false-negative direction resurrects the exact self-transcription bug the guard exists to close.
The flag matches Home Assistant's actual decision by construction.

## Re-syncing on an ESPHome bump

1. Bump `requirements.txt`, rebuild `.venv` (`scripts/setup_build_env.sh`).
2. Re-copy the component:
   `cp -R .venv/lib/python*/site-packages/esphome/components/voice_assistant/ esphome/components/voice_assistant/`
   (then delete `__pycache__`; keep this file).
3. Re-apply the `// FPH:` block in `voice_assistant.h` — `git diff` against the pre-copy state
   shows exactly what to restore; it is the only diff there should be.
4. Re-verify the flag's lifetime in the new upstream: set at `INTENT_END` and `on_announce`,
   reset only in `request_stop()` and per-turn reassignment, consumed after `on_end` — the
   contract `tts_routing.yaml`'s guard depends on.

## Exit plan

PR the getter upstream to esphome/esphome. Once released, delete this directory and the
`voice_assistant` entries in both `external_components` lists (`config/satellite1.yaml` and
`config/common/components.external.yaml`), and the guard keeps working against upstream unchanged.
