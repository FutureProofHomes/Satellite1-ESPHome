from esphome import automation
import esphome.codegen as cg
from esphome.components import audio, microphone, sensor, socket, text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MICROPHONE, CONF_TRIGGER_ID

from . import stt_wake_verifier_ns

DEPENDENCIES = ["esp32", "network"]
AUTO_LOAD = ["audio", "json", "sensor", "socket", "text_sensor"]

SttWakeVerifier = stt_wake_verifier_ns.class_(
    "SttWakeVerifier", microphone.Microphone, cg.Component
)
VerifiedTrigger = stt_wake_verifier_ns.class_(
    "VerifiedTrigger", automation.Trigger.template(cg.std_string)
)
CancelledTrigger = stt_wake_verifier_ns.class_("CancelledTrigger", automation.Trigger.template(cg.bool_))

CONFIG_SCHEMA = microphone.MICROPHONE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(SttWakeVerifier),
        cv.Required(CONF_MICROPHONE): microphone.microphone_source_schema(
            min_bits_per_sample=16, max_bits_per_sample=16,
            min_channels=1, max_channels=1,
        ),
        cv.Required("capture_allowed"): cv.returning_lambda,
        cv.Optional("command_preroll", default="250ms"): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(min=cv.TimePeriod(milliseconds=0), max=cv.TimePeriod(seconds=1)),
        ),
        cv.Optional("on_verified"): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(VerifiedTrigger)}
        ),
        cv.Optional("on_cancelled"): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CancelledTrigger)}
        ),
        cv.Optional("result"): text_sensor.text_sensor_schema(entity_category="diagnostic"),
        cv.Optional("transcript"): text_sensor.text_sensor_schema(entity_category="diagnostic"),
        cv.Optional("score"): sensor.sensor_schema(
            unit_of_measurement="%", accuracy_decimals=1, entity_category="diagnostic"
        ),
        cv.Optional("duration"): sensor.sensor_schema(
            unit_of_measurement="ms", accuracy_decimals=0, entity_category="diagnostic"
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


def _stream_limits(config):
    audio.set_stream_limits(
        min_bits_per_sample=16, max_bits_per_sample=16,
        min_channels=1, max_channels=1,
        min_sample_rate=16000, max_sample_rate=16000,
    )(config)
    return config


CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, _stream_limits, socket.consume_sockets(1, "stt_wake_verifier"))
FINAL_VALIDATE_SCHEMA = cv.Schema(
    {cv.Required(CONF_MICROPHONE): microphone.final_validate_microphone_source_schema(
        "stt_wake_verifier", sample_rate=16000
    )}, extra=cv.ALLOW_EXTRA,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await microphone.register_microphone(var, config)
    source = await microphone.microphone_source_to_code(config[CONF_MICROPHONE])
    cg.add(var.set_source(source))
    allowed = await cg.process_lambda(config["capture_allowed"], [], return_type=cg.bool_)
    cg.add(var.set_capture_allowed(allowed))
    cg.add(var.set_command_preroll(config["command_preroll"].total_milliseconds))
    for name in ("result", "transcript"):
        if name in config:
            entity = await text_sensor.new_text_sensor(config[name])
            cg.add(getattr(var, f"set_{name}_sensor")(entity))
    for name in ("score", "duration"):
        if name in config:
            entity = await sensor.new_sensor(config[name])
            cg.add(getattr(var, f"set_{name}_sensor")(entity))
    for conf in config.get("on_verified", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "wake_word")], conf)
    for conf in config.get("on_cancelled", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.bool_, "recording")], conf)
