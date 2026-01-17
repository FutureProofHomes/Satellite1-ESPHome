import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor
from esphome.const import CONF_ID

from ..satellite1 import (
    CONF_SATELLITE1,
    Satellite1,
    namespace as sat1_ns,
)

CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["satellite1"]

Satellite1DoA = sat1_ns.class_("Satellite1DoA", cg.PollingComponent)

CONF_AZIMUTH = "azimuth"
CONF_CONFIDENCE = "confidence"
CONF_DELAY_OFFSETS_SAMPLES = "delay_offsets_samples"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Satellite1DoA),
        cv.GenerateID(CONF_SATELLITE1): cv.use_id(Satellite1),
        cv.Required(CONF_AZIMUTH): sensor.sensor_schema(unit_of_measurement="°", accuracy_decimals=1),
        cv.Optional(CONF_CONFIDENCE): sensor.sensor_schema(unit_of_measurement="", accuracy_decimals=2),
        cv.Optional(CONF_DELAY_OFFSETS_SAMPLES): cv.All(
            cv.ensure_list(cv.float_),
            cv.Length(min=4, max=4),
        ),
    }
).extend(cv.polling_component_schema("500ms"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_SATELLITE1])

    az = await sensor.new_sensor(config[CONF_AZIMUTH])
    cg.add(var.set_azimuth_sensor(az))

    if CONF_CONFIDENCE in config:
        conf = await sensor.new_sensor(config[CONF_CONFIDENCE])
        cg.add(var.set_confidence_sensor(conf))

    if CONF_DELAY_OFFSETS_SAMPLES in config:
        cg.add(var.set_delay_offsets(config[CONF_DELAY_OFFSETS_SAMPLES]))

    return var
