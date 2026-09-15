"""Maps the generated main.cpp's zero-initialized statics into PSRAM.

ESPHome 2026.x places every component object in static storage in the generated main.cpp
(the *__pstorage symbols), which lands in internal DRAM .bss - ~45KB on this firmware, all of it
task-context state. An ESP-IDF linker fragment moves it to PSRAM's .ext_ram.bss, minus the few
objects that embed FreeRTOS TCBs (see linker.lf).

Safety: PSRAM access is only fatal while the flash cache is disabled, and only IRAM-flagged
interrupts run then. None of the ISRs that touch component objects (GPIO, RMT, I2S) are
IRAM-flagged on this build, so they are masked during those windows. .ext_ram.bss is zeroed by
esp_psram init, well before any ESPHome code runs. Verified with an audio + wake word + OTA soak.
"""

from pathlib import Path

import esphome.config_validation as cv
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option

DEPENDENCIES = ["esp32"]

CONFIG_SCHEMA = cv.Schema({})


async def to_code(config):
    # Enables the .ext_ram.bss output section in the generated linker script.
    add_idf_sdkconfig_option("CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY", True)
    # A source-less IDF component that only carries the linker fragment; fragments are global,
    # so it may map another component's archive. Directory name must match the component name.
    add_idf_component(
        name="sat1_extram_bss",
        path=str((Path(__file__).parent / "sat1_extram_bss").resolve()),
    )
