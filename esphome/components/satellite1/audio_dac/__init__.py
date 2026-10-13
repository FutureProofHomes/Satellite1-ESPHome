import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components.audio_dac import AudioDac, audio_dac_ns
from esphome.components.pcm5122.audio_dac import PCM5122
from esphome.components.tas2780.audio_dac import tas2780

from esphome.const import CONF_ID, CONF_OUTPUT, CONF_TRIGGER_ID

from ..satellite1 import (
    Satellite1, 
    Satellite1SPIService,
    namespace as sat1_ns,
    CONF_SATELLITE1
)


CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["satellite1"]
AUTO_LOAD = ["speaker_source"]

DACProxy = sat1_ns.class_("DACProxy", AudioDac, Satellite1SPIService, cg.Component)
VolumeState = cg.esphome_ns.namespace("speaker_source").class_("VolumeState")
DACOutputVolumeState = sat1_ns.class_("DACOutputVolumeState", VolumeState)
DacOutput = sat1_ns.enum("DacOutput")
DAC_OUTPUTS = {"speaker": DacOutput.SPEAKER, "line_out": DacOutput.LINE_OUT}
SetVolumeLimitsAction = sat1_ns.class_(
    "SetVolumeLimitsAction", automation.Action, cg.Parented.template(DACProxy)
)

ActivateAction = sat1_ns.class_(
    "ActivateAction", automation.Action, cg.Parented.template(DACProxy)
)


ActivateLineOutAction = sat1_ns.class_(
    "ActivateLineOutAction", automation.Action, cg.Parented.template(DACProxy)
)

ActivateSpeakerAction = sat1_ns.class_(
    "ActivateSpeakerAction", automation.Action, cg.Parented.template(DACProxy)
)

StateTrigger = sat1_ns.class_("StateTrigger", automation.Trigger)
SpeakerActivatedTrigger = sat1_ns.class_("SpeakerActivatedTrigger", automation.Trigger.template())
LineOutActivatedTrigger = sat1_ns.class_("LineOutActivatedTrigger", automation.Trigger.template())



CONF_SPEAKER_DAC = "speaker_dac"
CONF_LINE_OUT_DAC = "line_out_dac" 
CONF_VOLUME_STATE = "volume_state"
CONF_MIN_DB = "min_db"
CONF_MAX_DB = "max_db"

CONF_ON_STATE_CHANGE = "on_state_change"
CONF_ON_SPEAKER_ACTIVATED = "on_speaker_activated"
CONF_ON_LINE_OUT_ACTIVATED = "on_line_out_activated"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DACProxy),
            cv.GenerateID(CONF_SATELLITE1): cv.use_id(Satellite1),
            cv.Optional(CONF_SPEAKER_DAC) : cv.use_id(tas2780),
            cv.Optional(CONF_LINE_OUT_DAC) : cv.use_id(PCM5122),
            cv.Required(CONF_VOLUME_STATE): cv.Schema(
                {cv.GenerateID(): cv.declare_id(DACOutputVolumeState)}
            ),

            cv.Optional(CONF_ON_STATE_CHANGE): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(StateTrigger),
            }),
            cv.Optional(CONF_ON_SPEAKER_ACTIVATED): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SpeakerActivatedTrigger),
            }),
            cv.Optional(CONF_ON_LINE_OUT_ACTIVATED): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LineOutActivatedTrigger),
            }),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_SATELLITE1])
    if CONF_SPEAKER_DAC in config :
        speaker_dac = await cg.get_variable(config[CONF_SPEAKER_DAC])
        cg.add( var.set_speaker_dac(speaker_dac) )
        cg.add(var.set_volume_limits_callback(DacOutput.SPEAKER, cg.RawExpression(
            f"[=](float min_db, float max_db) {{ "
            f"{speaker_dac}->set_vol_range_min((min_db + 100.0f) / 100.0f); "
            f"{speaker_dac}->set_vol_range_max((max_db + 100.0f) / 100.0f); }}"
        )))
    if CONF_LINE_OUT_DAC in config :
        lineout_dac = await cg.get_variable(config[CONF_LINE_OUT_DAC])
        cg.add( var.set_lineout_dac(lineout_dac) )
        cg.add(var.set_volume_limits_callback(DacOutput.LINE_OUT, cg.RawExpression(
            f"[=](float min_db, float max_db) {{ "
            f"{lineout_dac}->set_volume_min_db(min_db); "
            f"{lineout_dac}->set_volume_max_db(max_db); }}"
        )))
    
    volume_state = cg.new_Pvariable(config[CONF_VOLUME_STATE][CONF_ID])
    cg.add(volume_state.set_parent(var))

    for conf in config.get(CONF_ON_STATE_CHANGE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_SPEAKER_ACTIVATED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_LINE_OUT_ACTIVATED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


DAC_ACTION_SCHEMA = automation.maybe_simple_id({cv.GenerateID(): cv.use_id(DACProxy)})

@automation.register_action(
    "dac_proxy.set_volume_limits",
    SetVolumeLimitsAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(DACProxy),
        cv.Required(CONF_OUTPUT): cv.templatable(cv.enum(DAC_OUTPUTS, lower=True)),
        cv.Required(CONF_MIN_DB): cv.templatable(cv.decibel),
        cv.Required(CONF_MAX_DB): cv.templatable(cv.decibel),
    }),
    synchronous=True,
)
async def set_volume_limits_action(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    for key, type_ in ((CONF_OUTPUT, DacOutput), (CONF_MIN_DB, cg.float_), (CONF_MAX_DB, cg.float_)):
        value = await cg.templatable(config[key], args, type_)
        cg.add(getattr(var, f"set_{key}")(value))
    return var

@automation.register_action("dac_proxy.activate", ActivateAction, DAC_ACTION_SCHEMA, synchronous=True)
@automation.register_action("dac_proxy.activate_line_out", ActivateLineOutAction, DAC_ACTION_SCHEMA, synchronous=True)
@automation.register_action("dac_proxy.activate_speaker", ActivateSpeakerAction, DAC_ACTION_SCHEMA, synchronous=True)
async def tas2780_action(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
