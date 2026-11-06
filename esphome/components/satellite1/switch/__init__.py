import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import binary_sensor, switch
from esphome.const import CONF_TRIGGER_ID
from ..satellite1 import Satellite1, namespace

DEPENDENCIES = ["satellite1"]
AUTO_LOAD = ["binary_sensor"]

MicrophoneMuteSwitch = namespace.class_("MicrophoneMuteSwitch", switch.Switch, cg.Component)
HardwareMuteChangeTrigger = namespace.class_("HardwareMuteChangeTrigger", automation.Trigger.template(cg.bool_))
UnmuteRejectedTrigger = namespace.class_("UnmuteRejectedTrigger", automation.Trigger)

CONFIG_SCHEMA = switch.switch_schema(MicrophoneMuteSwitch, block_inverted=True).extend({
    cv.Optional("restore_mode", default="DISABLED"): cv.enum(
        {"DISABLED": switch.RESTORE_MODES["DISABLED"]}, upper=True
    ),
    cv.GenerateID("satellite1_id"): cv.use_id(Satellite1),
    cv.Required("hardware_mute"): binary_sensor.binary_sensor_schema(),
    cv.Optional("on_hardware_mute_change"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(HardwareMuteChangeTrigger),
    }),
    cv.Optional("on_unmute_rejected"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(UnmuteRejectedTrigger),
    }),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    parent = await cg.get_variable(config["satellite1_id"])
    cg.add(var.set_parent(parent))
    hardware = await binary_sensor.new_binary_sensor(config["hardware_mute"])
    cg.add(var.set_hardware_sensor(hardware))
    for conf in config.get("on_hardware_mute_change", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.bool_, "muted")], conf)
    for conf in config.get("on_unmute_rejected", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
