import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import automation
from esphome.components import audio, esp32, microphone
from esphome.const import CONF_ID, CONF_MICROPHONE, CONF_ON_TIMEOUT


AUTO_LOAD = ["audio"]
DEPENDENCIES = ["microphone"]

CONF_MEDIA_FILE = "media_file"
CONF_CHANNEL = "channel"
CONF_ON_DETECTED = "on_detected"
CONF_UDP_STREAM_ENABLED = "udp_stream_enabled"
CONF_UDP_STREAM_HOST = "udp_stream_host"
CONF_UDP_STREAM_PORT = "udp_stream_port"
CONF_UDP_STREAM_PACKET_SAMPLES = "udp_stream_packet_samples"

online_testing_ns = cg.esphome_ns.namespace("online_testing")
MicTester = online_testing_ns.class_("MicTester", cg.Component)

StartAction = online_testing_ns.class_(
    "StartAction", automation.Action, cg.Parented.template(MicTester)
)
StartContinuousAction = online_testing_ns.class_(
    "StartContinuousAction", automation.Action, cg.Parented.template(MicTester)
)
StopAction = online_testing_ns.class_(
    "StopAction", automation.Action, cg.Parented.template(MicTester)
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MicTester),
            cv.GenerateID(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
            cv.Required(CONF_MEDIA_FILE): cv.use_id(audio.AudioFile),
            cv.Optional(CONF_CHANNEL, default=0): cv.int_range(min=0, max=1),
            cv.Optional(CONF_ON_DETECTED): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_TIMEOUT): automation.validate_automation(single=True),
            cv.Optional(CONF_UDP_STREAM_ENABLED, default=False): cv.boolean,
            cv.Optional(CONF_UDP_STREAM_HOST): cv.string,
            cv.Optional(CONF_UDP_STREAM_PORT, default=5005): cv.port,
            cv.Optional(CONF_UDP_STREAM_PACKET_SAMPLES, default=960): cv.positive_int,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


def _validate_microphone(config):
    return audio.final_validate_audio_schema(
        "online_testing",
        audio_device=CONF_MICROPHONE,
        sample_rate=16000,
        enabled_channels=[config[CONF_CHANNEL]],
    )(config)


FINAL_VALIDATE_SCHEMA = _validate_microphone


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
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(var.set_udp_stream_enabled(config[CONF_UDP_STREAM_ENABLED]))
    cg.add(var.set_udp_stream_port(config[CONF_UDP_STREAM_PORT]))
    cg.add(var.set_udp_stream_packet_samples(config[CONF_UDP_STREAM_PACKET_SAMPLES]))
    if CONF_UDP_STREAM_HOST in config:
        cg.add(var.set_udp_stream_host(config[CONF_UDP_STREAM_HOST]))

    if CONF_ON_DETECTED in config:
        await automation.build_automation(
            var.get_sweep_detected_trigger(), [], config[CONF_ON_DETECTED]
        )
    if CONF_ON_TIMEOUT in config:
        await automation.build_automation(var.get_end_trigger(), [], config[CONF_ON_TIMEOUT])


MIC_TESTER_ACTION_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(MicTester)})


@automation.register_action(
    "mic_tester.start_continuous",
    StartContinuousAction,
    MIC_TESTER_ACTION_SCHEMA,
    synchronous=False,
)
@automation.register_action(
    "mic_tester.start", StartAction, MIC_TESTER_ACTION_SCHEMA, synchronous=False
)
async def mic_tester_start_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "mic_tester.stop", StopAction, MIC_TESTER_ACTION_SCHEMA, synchronous=False
)
async def mic_tester_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
