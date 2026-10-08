"""OpenAI Realtime API over WebSockets: a full-duplex voice conversation straight from the device.

The component owns one WebSocket session per conversation. The microphone (16 kHz, the XMOS
echo-cancelled channel) is upsampled to 24 kHz PCM and streamed up; the model's 24 kHz PCM is played
through a speaker (normally a resampler feeding its own mixer input). The server's voice activity
detection decides turns, and talking over the model interrupts it: playback stops at once and the
unheard part of the answer is truncated from the conversation.

Credentials (base URL, API key, model, voice, enabled) are stored on the device and edited in the
web UI (Settings > OpenAI); the YAML values below are only the defaults a fresh device starts with.
See docs/openai-realtime.md.
"""

import json

from esphome import automation
import esphome.codegen as cg
from esphome.components import esp32, microphone, socket, speaker
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MICROPHONE,
    CONF_MODEL,
    CONF_NAME,
    CONF_ON_ERROR,
    CONF_SPEAKER,
    CONF_THRESHOLD,
    CONF_TYPE,
)

CODEOWNERS = []
DEPENDENCIES = ["network", "microphone", "speaker"]
AUTO_LOAD = ["audio", "ring_buffer", "socket", "json"]

openai_realtime_ns = cg.esphome_ns.namespace("openai_realtime")
OpenAIRealtime = openai_realtime_ns.class_("OpenAIRealtime", cg.Component)

StartAction = openai_realtime_ns.class_(
    "StartAction", automation.Action, cg.Parented.template(OpenAIRealtime)
)
StopAction = openai_realtime_ns.class_(
    "StopAction", automation.Action, cg.Parented.template(OpenAIRealtime)
)
FunctionResultAction = openai_realtime_ns.class_(
    "FunctionResultAction", automation.Action, cg.Parented.template(OpenAIRealtime)
)
SendTextAction = openai_realtime_ns.class_(
    "SendTextAction", automation.Action, cg.Parented.template(OpenAIRealtime)
)
IsActiveCondition = openai_realtime_ns.class_(
    "IsActiveCondition", automation.Condition, cg.Parented.template(OpenAIRealtime)
)
IsEnabledCondition = openai_realtime_ns.class_(
    "IsEnabledCondition", automation.Condition, cg.Parented.template(OpenAIRealtime)
)

CONF_BASE_URL = "base_url"
CONF_API_KEY = "api_key"
CONF_VOICE = "voice"
CONF_ENABLED = "enabled"
CONF_INSTRUCTIONS = "instructions"
CONF_TURN_DETECTION = "turn_detection"
CONF_EAGERNESS = "eagerness"
CONF_PREFIX_PADDING = "prefix_padding"
CONF_SILENCE_DURATION = "silence_duration"
CONF_NOISE_REDUCTION = "noise_reduction"
CONF_INPUT_TRANSCRIPTION_MODEL = "input_transcription_model"
CONF_TOOLS = "tools"
CONF_DESCRIPTION = "description"
CONF_PARAMETERS = "parameters"
CONF_END_CONVERSATION_TOOL = "end_conversation_tool"
CONF_HALF_DUPLEX = "half_duplex"
CONF_API_VERSION = "api_version"
CONF_IDLE_TIMEOUT = "idle_timeout"
CONF_MAX_DURATION = "max_duration"
CONF_PLAYBACK_LATENCY = "playback_latency"
CONF_PLAYBACK_BUFFER_SIZE = "playback_buffer_size"
CONF_EXTRA_HEADERS = "extra_headers"
CONF_MUTED = "muted"

CONF_ON_START = "on_start"
CONF_ON_READY = "on_ready"
CONF_ON_LISTENING = "on_listening"
CONF_ON_SPEECH_STARTED = "on_speech_started"
CONF_ON_SPEECH_STOPPED = "on_speech_stopped"
CONF_ON_RESPONSE_STARTED = "on_response_started"
CONF_ON_RESPONSE_FINISHED = "on_response_finished"
CONF_ON_INTERRUPTED = "on_interrupted"
CONF_ON_END = "on_end"
CONF_ON_USER_TRANSCRIPT = "on_user_transcript"
CONF_ON_ASSISTANT_TRANSCRIPT = "on_assistant_transcript"
CONF_ON_FUNCTION_CALL = "on_function_call"

CONF_CALL_ID = "call_id"
CONF_OUTPUT = "output"
CONF_RESPOND = "respond"
CONF_TEXT = "text"



def _base_url(value):
    value = cv.string_strict(value).strip()
    if not value.lower().startswith(("http://", "https://", "ws://", "wss://")):
        raise cv.Invalid("base_url must start with https://, http://, wss:// or ws://")
    if len(value) > 159:
        raise cv.Invalid("base_url is limited to 159 characters")
    return value


def _token(max_len):
    def validator(value):
        value = cv.string_strict(value)
        if not value or len(value) > max_len:
            raise cv.Invalid(f"must be 1 to {max_len} characters")
        return value

    return validator


def _tool_name(value):
    value = cv.string_strict(value)
    if not value or len(value) > 64 or not all(c.isalnum() or c in "_-" for c in value):
        raise cv.Invalid("tool names are 1-64 letters, digits, '_' or '-'")
    if value == "end_conversation":
        raise cv.Invalid("end_conversation is built in; use end_conversation_tool instead")
    return value


def _parameters(value):
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError as err:
            raise cv.Invalid(f"parameters is not valid JSON: {err}") from err
    if not isinstance(value, dict):
        raise cv.Invalid("parameters must be a JSON schema object")
    return value


def _no_control_chars(value, path="tool"):
    """Tool definitions are copied verbatim into session.update; JSON text cannot carry raw control
    characters other than tab, newline and carriage return, so refuse them here at build time."""
    if isinstance(value, str):
        bad = [c for c in value if ord(c) < 0x20 and c not in "\t\n\r"]
        if bad:
            raise cv.Invalid(f"{path} contains a control character ({ord(bad[0]):#04x})")
    elif isinstance(value, dict):
        for k, v in value.items():
            _no_control_chars(k, path)
            _no_control_chars(v, f"{path}.{k}")
    elif isinstance(value, list):
        for i, v in enumerate(value):
            _no_control_chars(v, f"{path}[{i}]")
    return value


TOOL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NAME): _tool_name,
        cv.Required(CONF_DESCRIPTION): cv.string,
        cv.Optional(
            CONF_PARAMETERS, default={"type": "object", "properties": {}}
        ): _parameters,
    }
).add_extra(_no_control_chars)

TURN_DETECTION_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TYPE, default="semantic_vad"): cv.one_of(
            "semantic_vad", "server_vad", "none", lower=True
        ),
        cv.Optional(CONF_EAGERNESS, default="auto"): cv.one_of(
            "auto", "low", "medium", "high", lower=True
        ),
        cv.Optional(CONF_THRESHOLD, default=0.5): cv.float_range(min=0.0, max=1.0),
        cv.Optional(
            CONF_PREFIX_PADDING, default="300ms"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_SILENCE_DURATION, default="500ms"
        ): cv.positive_time_period_milliseconds,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OpenAIRealtime),
        # Defaults only - the web UI's saved values win once something has been saved.
        cv.Optional(CONF_ENABLED, default=False): cv.boolean,
        cv.Optional(CONF_BASE_URL, default="https://api.openai.com/v1"): _base_url,
        cv.Optional(CONF_API_KEY, default=""): cv.All(cv.string, cv.Length(max=255)),
        cv.Optional(CONF_MODEL, default="gpt-realtime-2.1"): _token(63),
        cv.Optional(CONF_VOICE, default="marin"): _token(63),
        cv.Optional(CONF_INSTRUCTIONS, default=""): cv.string,
        cv.Optional(CONF_TURN_DETECTION, default={}): TURN_DETECTION_SCHEMA,
        cv.Optional(CONF_NOISE_REDUCTION, default="none"): cv.one_of(
            "none", "near_field", "far_field", lower=True
        ),
        cv.Optional(CONF_INPUT_TRANSCRIPTION_MODEL, default=""): cv.string,
        cv.Optional(CONF_TOOLS, default=[]): cv.ensure_list(TOOL_SCHEMA),
        cv.Optional(CONF_END_CONVERSATION_TOOL, default=True): cv.boolean,
        cv.Optional(CONF_HALF_DUPLEX, default=False): cv.boolean,
        # Wire format: "auto" reads it from the server's session.created (GA for api.openai.com,
        # usually beta for self-hosted servers); force one only for a server that sends none.
        cv.Optional(CONF_API_VERSION, default="auto"): cv.one_of(
            "auto", "ga", "beta", lower=True
        ),
        cv.Optional(
            CONF_IDLE_TIMEOUT, default="20s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_MAX_DURATION, default="15min"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_PLAYBACK_LATENCY, default="600ms"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_PLAYBACK_BUFFER_SIZE, default=786432): cv.int_range(
            min=65536, max=4 * 1024 * 1024
        ),
        cv.Optional(CONF_EXTRA_HEADERS, default={}): cv.Schema(
            {cv.string_strict: cv.string_strict}
        ),
        cv.Required(CONF_MICROPHONE): microphone.microphone_source_schema(
            min_bits_per_sample=16, max_bits_per_sample=16, min_channels=1, max_channels=1
        ),
        cv.Required(CONF_SPEAKER): cv.use_id(speaker.Speaker),
        cv.Optional(CONF_MUTED): cv.returning_lambda,
        cv.Optional(CONF_ON_START): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_READY): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_LISTENING): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_SPEECH_STARTED): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_SPEECH_STOPPED): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_RESPONSE_STARTED): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_RESPONSE_FINISHED): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_INTERRUPTED): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_END): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_ERROR): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_USER_TRANSCRIPT): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_ASSISTANT_TRANSCRIPT): automation.validate_automation(
            single=True
        ),
        cv.Optional(CONF_ON_FUNCTION_CALL): automation.validate_automation(single=True),
    }
).extend(cv.COMPONENT_SCHEMA)


def _final_validate(config):
    # The Realtime path is 16 kHz in (resampled here to 24 kHz), like voice_assistant.
    microphone.final_validate_microphone_source_schema(
        "openai_realtime", sample_rate=16000
    )(config[CONF_MICROPHONE])
    return config


FINAL_VALIDATE_SCHEMA = _final_validate

CONFIG_SCHEMA = cv.All(
    CONFIG_SCHEMA,
    cv.only_on_esp32,
    # One outbound TCP connection per session, plus one for the /models fetch.
    socket.consume_sockets(2, "openai_realtime"),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic_source = await microphone.microphone_source_to_code(config[CONF_MICROPHONE])
    cg.add(var.set_microphone_source(mic_source))
    spk = await cg.get_variable(config[CONF_SPEAKER])
    cg.add(var.set_speaker(spk))

    cg.add(var.set_default_enabled(config[CONF_ENABLED]))
    cg.add(var.set_default_base_url(config[CONF_BASE_URL]))
    cg.add(var.set_default_api_key(config[CONF_API_KEY]))
    cg.add(var.set_default_model(config[CONF_MODEL]))
    cg.add(var.set_default_voice(config[CONF_VOICE]))
    cg.add(var.set_instructions(config[CONF_INSTRUCTIONS]))

    td = config[CONF_TURN_DETECTION]
    cg.add(
        var.set_turn_detection(
            td[CONF_TYPE],
            td[CONF_EAGERNESS],
            td[CONF_THRESHOLD],
            td[CONF_PREFIX_PADDING].total_milliseconds,
            td[CONF_SILENCE_DURATION].total_milliseconds,
        )
    )
    cg.add(var.set_noise_reduction(config[CONF_NOISE_REDUCTION]))
    cg.add(var.set_input_transcription_model(config[CONF_INPUT_TRANSCRIPTION_MODEL]))

    tools = [
        {
            "type": "function",
            "name": t[CONF_NAME],
            "description": t[CONF_DESCRIPTION],
            "parameters": t[CONF_PARAMETERS],
        }
        for t in config[CONF_TOOLS]
    ]
    if tools:
        cg.add(var.set_tools_json(json.dumps(tools, separators=(",", ":"))))
    cg.add(var.set_end_conversation_tool(config[CONF_END_CONVERSATION_TOOL]))
    cg.add(var.set_half_duplex(config[CONF_HALF_DUPLEX]))
    cg.add(var.set_api_version(config[CONF_API_VERSION]))
    cg.add(var.set_idle_timeout(config[CONF_IDLE_TIMEOUT].total_milliseconds))
    cg.add(var.set_max_duration(config[CONF_MAX_DURATION].total_milliseconds))
    cg.add(var.set_playback_latency(config[CONF_PLAYBACK_LATENCY].total_milliseconds))
    cg.add(var.set_playback_buffer_size(config[CONF_PLAYBACK_BUFFER_SIZE]))
    for key, value in config[CONF_EXTRA_HEADERS].items():
        cg.add(var.set_extra_header(key, value))

    if CONF_MUTED in config:
        muted = await cg.process_lambda(config[CONF_MUTED], [], return_type=cg.bool_)
        cg.add(var.set_muted_lambda(muted))

    simple = [
        (CONF_ON_START, var.get_start_trigger()),
        (CONF_ON_READY, var.get_ready_trigger()),
        (CONF_ON_LISTENING, var.get_listening_trigger()),
        (CONF_ON_SPEECH_STARTED, var.get_speech_started_trigger()),
        (CONF_ON_SPEECH_STOPPED, var.get_speech_stopped_trigger()),
        (CONF_ON_RESPONSE_STARTED, var.get_response_started_trigger()),
        (CONF_ON_RESPONSE_FINISHED, var.get_response_finished_trigger()),
        (CONF_ON_INTERRUPTED, var.get_interrupted_trigger()),
        (CONF_ON_END, var.get_end_trigger()),
    ]
    for key, trigger in simple:
        if key in config:
            await automation.build_automation(trigger, [], config[key])
    if CONF_ON_ERROR in config:
        await automation.build_automation(
            var.get_error_trigger(),
            [(cg.std_string, "code"), (cg.std_string, "message")],
            config[CONF_ON_ERROR],
        )
    if CONF_ON_USER_TRANSCRIPT in config:
        await automation.build_automation(
            var.get_user_transcript_trigger(),
            [(cg.std_string, "x")],
            config[CONF_ON_USER_TRANSCRIPT],
        )
    if CONF_ON_ASSISTANT_TRANSCRIPT in config:
        await automation.build_automation(
            var.get_assistant_transcript_trigger(),
            [(cg.std_string, "x")],
            config[CONF_ON_ASSISTANT_TRANSCRIPT],
        )
    if CONF_ON_FUNCTION_CALL in config:
        cg.add(var.set_has_function_handler(True))
        await automation.build_automation(
            var.get_function_call_trigger(),
            [
                (cg.std_string, "name"),
                (cg.std_string, "arguments"),
                (cg.std_string, "call_id"),
            ],
            config[CONF_ON_FUNCTION_CALL],
        )

    # esp_websocket_client from the component registry; esp-tls, tcp_transport and esp_http_client
    # are built-in IDF components ESPHome excludes unless a component asks for them.
    esp32.add_idf_component(name="espressif/esp_websocket_client", ref="1.6.0")
    for name in ("esp-tls", "tcp_transport", "esp_http_client"):
        esp32.include_builtin_idf_component(name)
    esp32.require_certificate_bundle()
    cg.add_define("USE_OPENAI_REALTIME")


OPENAI_REALTIME_ACTION_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(OpenAIRealtime)})


@automation.register_action(
    "openai_realtime.start", StartAction, OPENAI_REALTIME_ACTION_SCHEMA, synchronous=True
)
@automation.register_action(
    "openai_realtime.stop", StopAction, OPENAI_REALTIME_ACTION_SCHEMA, synchronous=True
)
async def openai_realtime_simple_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "openai_realtime.send_function_result",
    FunctionResultAction,
    OPENAI_REALTIME_ACTION_SCHEMA.extend(
        {
            cv.Required(CONF_CALL_ID): cv.templatable(cv.string),
            cv.Required(CONF_OUTPUT): cv.templatable(cv.string),
            cv.Optional(CONF_RESPOND, default=True): cv.templatable(cv.boolean),
        }
    ),
    synchronous=True,
)
async def openai_realtime_function_result_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_call_id(await cg.templatable(config[CONF_CALL_ID], args, cg.std_string)))
    cg.add(var.set_output(await cg.templatable(config[CONF_OUTPUT], args, cg.std_string)))
    cg.add(var.set_respond(await cg.templatable(config[CONF_RESPOND], args, cg.bool_)))
    return var


@automation.register_action(
    "openai_realtime.send_text",
    SendTextAction,
    OPENAI_REALTIME_ACTION_SCHEMA.extend(
        {cv.Required(CONF_TEXT): cv.templatable(cv.string)}
    ),
    synchronous=True,
)
async def openai_realtime_send_text_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_text(await cg.templatable(config[CONF_TEXT], args, cg.std_string)))
    return var


@automation.register_condition(
    "openai_realtime.is_active", IsActiveCondition, OPENAI_REALTIME_ACTION_SCHEMA
)
@automation.register_condition(
    "openai_realtime.is_enabled", IsEnabledCondition, OPENAI_REALTIME_ACTION_SCHEMA
)
async def openai_realtime_condition_to_code(config, condition_id, template_arg, args):
    var = cg.new_Pvariable(condition_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
