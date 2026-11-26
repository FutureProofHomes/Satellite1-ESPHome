import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components.i2s_audio.speaker import I2SAudioSpeaker
from esphome.components.satellite1.satellite1 import Satellite1
from esphome.const import CONF_ID, CONF_TRIGGER_ID

DEPENDENCIES = ["satellite1", "i2s_audio"]

namespace = cg.esphome_ns.namespace("satellite1_directional_leds")
DirectionalLEDs = namespace.class_("DirectionalLEDs", cg.Component)
AvailabilityChangeTrigger = namespace.class_(
    "AvailabilityChangeTrigger", automation.Trigger.template()
)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DirectionalLEDs),
    cv.Required("satellite1_id"): cv.use_id(Satellite1),
    cv.Required("speaker_id"): cv.use_id(I2SAudioSpeaker),
    cv.Optional("on_availability_change"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(AvailabilityChangeTrigger),
    }),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    parent = await cg.get_variable(config["satellite1_id"])
    speaker = await cg.get_variable(config["speaker_id"])
    cg.add(var.set_satellite(parent))
    cg.add(var.set_speaker(speaker))
    for conf in config.get("on_availability_change", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
