"""Crash capture: what happened and when, across a spontaneous reboot.

Three layers, each answering a question the others cannot. ESP-IDF's core-dump-to-flash (the
`coredump` partition and sdkconfig options in config/common/core_board.yaml) records *where* it
crashed - task, PC, backtrace, a full ELF image. This component's RTC-memory flight recorder
records *when* and *what led up to it* - a minute-by-minute time mark and a ring of recent log
lines, both surviving every warm reset because RTC slow memory is not cleared by a panic reboot.
And an NVS ring of the last 8 crash records survives everything including power loss, so the
Diagnostics page still shows history after later clean reboots.

The web-facing half lives in satellite1_web_ui, which serves the records, the log tail and the
raw dump from /api/sat1/crash* when this component is in the build.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import logger, time
from esphome.const import CONF_ID, CONF_TIME_ID

CODEOWNERS = ["@FutureProofHomes"]
DEPENDENCIES = ["logger"]
MULTI_CONF = False

CONF_CAPTURE_LEVEL = "capture_level"

crash_report_ns = cg.esphome_ns.namespace("crash_report")
CrashReport = crash_report_ns.class_("CrashReport", cg.Component)

# esphome/core/log.h values. CONFIG (4) is deliberately above the INFO floor: it is boot-time
# configuration dumps, pages of them, and a ring that exists to hold the seconds before a crash
# should not spend itself on what the device printed at startup.
LOG_LEVELS = {
    "ERROR": 1,
    "WARN": 2,
    "INFO": 3,
    "DEBUG": 5,
    "VERBOSE": 6,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CrashReport),
        # Optional so a build without any time source still compiles; records then carry
        # uptime-at-crash and a boot counter, and the app words the moment from those.
        cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        cv.Optional(CONF_CAPTURE_LEVEL, default="INFO"): cv.one_of(
            *LOG_LEVELS, upper=True
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Compiles the logger's listener vector in (USE_LOG_LISTENERS), the same channel the wake word
    # tuner scores through - the flight recorder subscribes to every formatted log line.
    logger.request_log_listener()

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TIME_ID in config:
        cg.add(var.set_time(await cg.get_variable(config[CONF_TIME_ID])))

    cg.add(var.set_capture_level(LOG_LEVELS[config[CONF_CAPTURE_LEVEL]]))

    # satellite1_web_ui compiles its crash endpoints only when this component is in the build -
    # the same pattern as USE_SAT1_MWW_LOADER.
    cg.add_define("USE_SAT1_CRASH_REPORT", True)
