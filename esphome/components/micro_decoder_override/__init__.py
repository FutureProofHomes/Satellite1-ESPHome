"""Points the build at our patched micro-decoder fork instead of the registry release.

The speaker media player streams TTS through the `esphome/micro-decoder` IDF component,
whose reader thread is created and joined per playback. Every teardown mid-stream frees the
thread's per-thread lwIP select semaphore (ESP-IDF pthread TLS destructor) while lwIP's
select_check_waiters() may still hold a latched pointer to it - it signals waiters *outside*
SYS_ARCH_PROTECT - so in-flight TCP data for the torn-down stream makes the tcpip thread
signal a freed handle: `assert failed: xQueueGenericSend queue.c:936 (pxQueue)` in `tiT`.
Root-caused September 21 2026 from a Crash Reports core dump; this fleet's TTS-routing stop
fan-outs interrupt streams constantly, which is why it reproduced within minutes here while
stock configs rarely see it.

The fix is the git submodule at esphome/idf_components/micro-decoder: our fork of
esphome-libs/micro-decoder carrying the reader-exit-grace hotfix (and later the
persistent-reader-thread branch submitted upstream). This component swaps it in.

How the swap works: esphome.components.audio registers `esphome/micro-decoder` with
ref="0.4.0" in its to_code, and add_idf_component() keeps one registry entry per name,
last writer wins. This to_code runs at CoroPriority.FINAL, after every normal-priority
component, so its path registration replaces the version pin and the generated
idf_component.yml carries `path:` instead of `version:` - the IDF component manager then
uses the submodule and downloads nothing. The replacement logs a version-conflict warning
at compile time ("0.4.0 replaced by None"); that warning is this component doing its job.

Delete this component, the submodule, and their satellite1.yaml entries once the upstream
fix ships in a stock ESPHome release (tracked in fix_tts_crash_race_3ddff362.plan.md).
"""

from pathlib import Path

import esphome.config_validation as cv
from esphome.components.esp32 import add_idf_component
from esphome.core import CoroPriority, coroutine_with_priority

DEPENDENCIES = ["esp32"]

CONFIG_SCHEMA = cv.Schema({})


@coroutine_with_priority(CoroPriority.FINAL)
async def to_code(config):
    submodule = (Path(__file__).parents[2] / "idf_components" / "micro-decoder").resolve()
    if not (submodule / "idf_component.yml").is_file():
        raise cv.Invalid(
            "The micro-decoder submodule is not checked out at "
            f"{submodule}. Run: git submodule update --init"
        )
    add_idf_component(
        name="esphome/micro-decoder",
        path=str(submodule),
    )
