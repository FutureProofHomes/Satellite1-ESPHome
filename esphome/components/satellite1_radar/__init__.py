import gzip
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, Framework
from esphome.core.entity_helpers import (
    register_device_class,
    register_icon,
    register_unit_of_measurement,
)
from esphome.core import CORE, HexInt
import esphome.final_validate as fv

CODEOWNERS = ["@FutureProofHomes"]
DEPENDENCIES = ["network", "uart"]

# Auto-loaded rather than declared a dependency so that the missing-web_server case is reported by
# _final_validate below, which can say what to add, instead of by the generic dependency error.
# web_server auto-loads this too; both resolve to the same single instance.
AUTO_LOAD = ["web_server_base"]
MULTI_CONF = False

# No consume_sockets() call. The tuner used to run its own esp_http_server and so had to reserve a
# socket; it is now a handler on the shared web_server, which reserves its own.

# Headroom for entities that may be registered dynamically at runtime.
#
# These values reserve StaticVector capacity in App for post-detection
# registration of radar entities.
#
# No "switch" entry: the only one was "Radar Tuner WebUI", and the tuner is always mounted now.
RUNTIME_ENTITY_HEADROOM = {
    "binary_sensor": 1,
    "sensor": 6,
    "text_sensor": 5,
    "button": 3,
}

CONF_LD2410_HTML_ID = "ld2410_html_id"
CONF_LD2450_HTML_ID = "ld2450_html_id"
CONF_SATELLITE1_RADAR_ID = "satellite1_radar_id"

satellite1_radar_ns = cg.esphome_ns.namespace("satellite1_radar")
Satellite1Radar = satellite1_radar_ns.class_(
    "Satellite1Radar", cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Satellite1Radar),
            cv.GenerateID(CONF_LD2410_HTML_ID): cv.declare_id(cg.uint8),
            cv.GenerateID(CONF_LD2450_HTML_ID): cv.declare_id(cg.uint8),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    cv.only_with_framework(Framework.ESP_IDF),
)


def _final_validate(config):
    # The radar tuner is a handler on the shared server now, so web_server is a hard requirement
    # rather than something to work around. Without it global_web_server_base is never
    # constructed and setup() would dereference null - a boot loop, not a missing page. Caught
    # here so it is a config error with a fix in it instead.
    #
    # web_server_base is in DEPENDENCIES and so is auto-loaded, but it only provides the plumbing;
    # nothing creates the listener or the "Visit device" link unless web_server itself is present.
    if "web_server" not in fv.full_config.get():
        raise cv.Invalid(
            "satellite1_radar serves the radar tuner from ESPHome's shared web server, so a "
            "top-level 'web_server:' block is required. Add one on port 80."
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate

async def to_code(config):
    device_class_indices = {
        "distance": register_device_class("distance"),
        "illuminance": register_device_class("illuminance"),
        "occupancy": register_device_class("occupancy"),
        "motion": register_device_class("motion"),
    }
    unit_indices = {
        "centimeter": register_unit_of_measurement("cm"),
        "percent": register_unit_of_measurement("%"),
    }
    icon_indices = {
        "radar": register_icon("mdi:radar"),
        "chip": register_icon("mdi:chip"),
        "signal": register_icon("mdi:signal"),
        "motion_sensor": register_icon("mdi:motion-sensor"),
        "account_multiple": register_icon("mdi:account-multiple"),
        "account": register_icon("mdi:account"),
        "account_arrow_right": register_icon("mdi:account-arrow-right"),
        "tune_vertical": register_icon("mdi:tune-vertical"),
        "factory": register_icon("mdi:factory"),
        "restart": register_icon("mdi:restart"),
        "database_refresh": register_icon("mdi:database-refresh"),
    }

    if any(device_class_indices.values()):
        cg.add_define("USE_ENTITY_DEVICE_CLASS")
    if any(unit_indices.values()):
        cg.add_define("USE_ENTITY_UNIT_OF_MEASUREMENT")
    if any(icon_indices.values()):
        cg.add_define("USE_ENTITY_ICON")

    for platform_name, extra in RUNTIME_ENTITY_HEADROOM.items():
        CORE.platform_counts[platform_name] = (
            CORE.platform_counts.get(platform_name, 0) + extra
        )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # USE_WEBSERVER and USE_WEBSERVER_PORT used to be defined here, to make Home Assistant show a
    # "Visit device" link for a tuner that was not on ESPHome's web server at all. web_server now
    # defines both for real, and faking them alongside it would be a redefinition.

    cg.add(
        var.set_device_class_indices(
            device_class_indices["distance"],
            device_class_indices["illuminance"],
            device_class_indices["occupancy"],
            device_class_indices["motion"],
        )
    )
    cg.add(var.set_unit_indices(unit_indices["centimeter"], unit_indices["percent"]))
    cg.add(
        var.set_icon_indices(
            icon_indices["radar"],
            icon_indices["chip"],
            icon_indices["signal"],
            icon_indices["motion_sensor"],
            icon_indices["account_multiple"],
            icon_indices["account"],
            icon_indices["account_arrow_right"],
            icon_indices["tune_vertical"],
            icon_indices["factory"],
            icon_indices["restart"],
            icon_indices["database_refresh"],
        )
    )

    base_dir = Path(__file__).parent
    ld2410_path = base_dir / "tuner_ui" / "radar_tuner_ld2410.html"
    ld2450_path = base_dir / "tuner_ui" / "radar_tuner_ld2450.html"

    ld2410_gz = gzip.compress(ld2410_path.read_bytes(), compresslevel=9)
    ld2450_gz = gzip.compress(ld2450_path.read_bytes(), compresslevel=9)

    ld2410_progmem = cg.progmem_array(
        config[CONF_LD2410_HTML_ID],
        tuple(map(HexInt, ld2410_gz)),
    )
    ld2450_progmem = cg.progmem_array(
        config[CONF_LD2450_HTML_ID],
        tuple(map(HexInt, ld2450_gz)),
    )

    cg.add(var.set_ld2410_html(ld2410_progmem, len(ld2410_gz)))
    cg.add(var.set_ld2450_html(ld2450_progmem, len(ld2450_gz)))
    return var
