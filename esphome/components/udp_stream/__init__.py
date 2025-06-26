import esphome.config_validation as cv
import esphome.codegen as cg

from esphome.const import (
    CONF_ID,
    CONF_MICROPHONE,
    CONF_IP_ADDRESS
)
from esphome import automation
from esphome.automation import register_action, register_condition
from esphome.components import microphone
from esphome.components.network import IPAddress

import socket

AUTO_LOAD = ["socket"]
DEPENDENCIES = ["microphone"]

CODEOWNERS = ["@gnumpi"]

CONF_ON_END = "on_end"
CONF_ON_ERROR = "on_error"
CONF_ON_START = "on_start"

udp_stream_ns = cg.esphome_ns.namespace("udp_stream")
UDPStreamer = udp_stream_ns.class_("UDPStreamer", cg.Component)

StartAction = udp_stream_ns.class_(
    "StartAction", automation.Action, cg.Parented.template(UDPStreamer)
)
StartContinuousAction = udp_stream_ns.class_(
    "StartContinuousAction", automation.Action, cg.Parented.template(UDPStreamer)
)
StopAction = udp_stream_ns.class_(
    "StopAction", automation.Action, cg.Parented.template(UDPStreamer)
)
IsRunningCondition = udp_stream_ns.class_(
    "IsRunningCondition", automation.Condition, cg.Parented.template(UDPStreamer)
)

def get_local_ip() -> str | None :
    local_hostname = socket.gethostname()
    ip_addresses = socket.gethostbyname_ex(local_hostname)[2]
    filtered_ips = [ip for ip in ip_addresses if not ip.startswith("127.")]
    return filtered_ips[0] if len(filtered_ips) else None

def safe_ip(ip):
    if ip is None:
        return IPAddress(0, 0, 0, 0)
    return IPAddress(*ip.args)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UDPStreamer),
            cv.GenerateID(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
            cv.Optional(CONF_IP_ADDRESS, default="") : cv.ipv4, #get_local_ip()
            cv.Optional(CONF_ON_START): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_END): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_ERROR): automation.validate_automation(single=True),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    cg.add(var.set_microphone(mic))
    cg.add(var.set_remote_ip(safe_ip(config[CONF_IP_ADDRESS])))

    if CONF_ON_START in config:
        await automation.build_automation(
            var.get_start_trigger(), [], config[CONF_ON_START]
        )

    if CONF_ON_END in config:
        await automation.build_automation(
            var.get_end_trigger(), [], config[CONF_ON_END]
        )

    if CONF_ON_ERROR in config:
        await automation.build_automation(
            var.get_error_trigger(),
            [(cg.std_string, "code"), (cg.std_string, "message")],
            config[CONF_ON_ERROR],
        )



UDP_STREAMER_ACTION_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(UDPStreamer)})


@register_action(
    "udp_stream.start_continuous",
    StartContinuousAction,
    UDP_STREAMER_ACTION_SCHEMA,
)
@register_action(
    "udp_stream.start",
    StartAction,
    UDP_STREAMER_ACTION_SCHEMA
)
async def voice_assistant_listen_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@register_action("udp_stream.stop", StopAction, UDP_STREAMER_ACTION_SCHEMA)
async def voice_assistant_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@register_condition(
    "udp_stream.is_running", IsRunningCondition, UDP_STREAMER_ACTION_SCHEMA
)
async def voice_assistant_is_running_to_code(config, condition_id, template_arg, args):
    var = cg.new_Pvariable(condition_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


