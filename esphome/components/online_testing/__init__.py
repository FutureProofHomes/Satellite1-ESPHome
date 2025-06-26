import esphome.config_validation as cv
import esphome.codegen as cg

from esphome.const import (
    CONF_ID,
    CONF_MICROPHONE,
    CONF_ON_TIMEOUT,
)
from esphome import automation
from esphome.automation import register_action, register_condition
from esphome.components import audio, esp32, microphone


AUTO_LOAD = ["audio"]
DEPENDENCIES = ["microphone","media_player"]

CODEOWNERS = ["@gnumpi"]

CONF_MEDIA_FILE = "media_file"
CONF_ON_DETECTED = "on_detected"

udp_stream_ns = cg.esphome_ns.namespace("online_testing")
MicTester = udp_stream_ns.class_("MicTester", cg.Component)

StartAction = udp_stream_ns.class_(
    "StartAction", automation.Action, cg.Parented.template(MicTester)
)
StartContinuousAction = udp_stream_ns.class_(
    "StartContinuousAction", automation.Action, cg.Parented.template(MicTester)
)
StopAction = udp_stream_ns.class_(
    "StopAction", automation.Action, cg.Parented.template(MicTester)
)
IsRunningCondition = udp_stream_ns.class_(
    "IsRunningCondition", automation.Condition, cg.Parented.template(MicTester)
)



CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MicTester),
            cv.GenerateID(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
            cv.Required(CONF_MEDIA_FILE): cv.use_id(audio.AudioFile),
            cv.Optional(CONF_ON_DETECTED): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_TIMEOUT): automation.validate_automation(single=True),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    esp32.add_idf_component(
        name="esp-dsp",
        repo="https://github.com/kahrendt/esp-dsp",
        ref="no-round-dot-product",
    )
    
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    cg.add(var.set_microphone(mic))
    media_file = await cg.get_variable(config[CONF_MEDIA_FILE])
    cg.add(var.set_media_file(media_file))

    if CONF_ON_DETECTED in config:
        await automation.build_automation(
            var.get_sweep_detected_trigger(), [], config[CONF_ON_DETECTED]
        )

    if CONF_ON_TIMEOUT in config:
        await automation.build_automation(
            var.get_end_trigger(), [], config[CONF_ON_TIMEOUT]
        )

MIC_TESTER_ACTION_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(MicTester)})

@register_action(
    "mic_tester.start_continuous",
    StartContinuousAction,
    MIC_TESTER_ACTION_SCHEMA,
)
@register_action(
    "mic_tester.start",
    StartAction,
    MIC_TESTER_ACTION_SCHEMA
)
async def voice_assistant_listen_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@register_action("mic_tester.stop", StopAction, MIC_TESTER_ACTION_SCHEMA)
async def voice_assistant_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

