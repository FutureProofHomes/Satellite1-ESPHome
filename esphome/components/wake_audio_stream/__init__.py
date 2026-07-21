import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import register_action, register_condition
from esphome.const import CONF_ID, CONF_PORT

AUTO_LOAD = ["socket"]
CODEOWNERS = ["@constructorfleet"]

CONF_IP_ADDRESS = "ip_address"
CONF_BUFFER_DURATION = "buffer_duration"
CONF_ENABLED_ON_BOOT = "enabled_on_boot"

wake_audio_stream_ns = cg.esphome_ns.namespace("wake_audio_stream")
WakeAudioStream = wake_audio_stream_ns.class_("WakeAudioStream", cg.Component)

StartAction = wake_audio_stream_ns.class_(
    "StartAction", automation.Action, cg.Parented.template(WakeAudioStream)
)
StopAction = wake_audio_stream_ns.class_(
    "StopAction", automation.Action, cg.Parented.template(WakeAudioStream)
)
IsRunningCondition = wake_audio_stream_ns.class_(
    "IsRunningCondition", automation.Condition, cg.Parented.template(WakeAudioStream)
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WakeAudioStream),
            cv.Required(CONF_IP_ADDRESS): cv.ipv4address,
            cv.Optional(CONF_PORT, default=6056): cv.port,
            cv.Optional(
                CONF_BUFFER_DURATION, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ENABLED_ON_BOOT, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    octets = config[CONF_IP_ADDRESS].packed
    cg.add(var.set_remote_ip(octets[0], octets[1], octets[2], octets[3]))
    cg.add(var.set_remote_port(config[CONF_PORT]))
    cg.add(var.set_buffer_duration_ms(config[CONF_BUFFER_DURATION].total_milliseconds))
    cg.add(var.set_enabled(config[CONF_ENABLED_ON_BOOT]))


WAKE_AUDIO_STREAM_ACTION_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(WakeAudioStream)})


@register_action(
    "wake_audio_stream.start",
    StartAction,
    WAKE_AUDIO_STREAM_ACTION_SCHEMA,
    synchronous=True,
)
async def wake_audio_stream_start_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@register_action(
    "wake_audio_stream.stop",
    StopAction,
    WAKE_AUDIO_STREAM_ACTION_SCHEMA,
    synchronous=True,
)
async def wake_audio_stream_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@register_condition(
    "wake_audio_stream.is_running", IsRunningCondition, WAKE_AUDIO_STREAM_ACTION_SCHEMA
)
async def wake_audio_stream_is_running_to_code(config, condition_id, template_arg, args):
    var = cg.new_Pvariable(condition_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
