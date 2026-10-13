"""OpenAI Realtime API over WebSockets: a full-duplex voice conversation straight from the device.

The component owns one WebSocket session per conversation. The microphone (16 kHz, the XMOS
echo-cancelled channel) is upsampled to 24 kHz PCM and streamed up; the model's 24 kHz PCM is played
through a speaker (normally a resampler feeding its own mixer input). The server's voice activity
detection decides turns, and talking over the model interrupts it: playback stops at once and the
unheard part of the answer is truncated from the conversation.

Everything is configured here in YAML: the base URL (OpenAI, or any compatible server - including a
self-hosted one on the local network), the API key, the model and the voice. The wire format (GA or
the older beta one most self-hosted servers speak) is detected from the server's first message.
See docs/openai-realtime.md and config/common/openai_realtime.yaml.
"""

import json
import re

from esphome import automation
import esphome.codegen as cg
from esphome.components import esp32, microphone, psram, socket, speaker
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
DEPENDENCIES = ["network", "microphone", "speaker", "psram"]
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
SetFullDuplexAction = openai_realtime_ns.class_(
    "SetFullDuplexAction", automation.Action, cg.Parented.template(OpenAIRealtime)
)
SendTextAction = openai_realtime_ns.class_(
    "SendTextAction", automation.Action, cg.Parented.template(OpenAIRealtime)
)
IsActiveCondition = openai_realtime_ns.class_(
    "IsActiveCondition", automation.Condition, cg.Parented.template(OpenAIRealtime)
)

CONF_BASE_URL = "base_url"
CONF_API_KEY = "api_key"
CONF_VOICE = "voice"
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
CONF_FULL_DUPLEX = "full_duplex"
CONF_API_VERSION = "api_version"
CONF_IDLE_TIMEOUT = "idle_timeout"
CONF_MAX_DURATION = "max_duration"
CONF_PLAYBACK_LATENCY = "playback_latency"
CONF_PLAYBACK_BUFFER_SIZE = "playback_buffer_size"
CONF_EXTRA_HEADERS = "extra_headers"
CONF_CA_CERTIFICATE = "ca_certificate"
CONF_VERIFY_CERTIFICATE_NAME = "verify_certificate_name"
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



OPENAI_HOST = "api.openai.com"


def _split_base_url(value):
    """Mirrors rt_util.cpp derive_endpoints(): (secure, host, path, query) or raises cv.Invalid."""
    m = re.match(r"^(https?|wss?)://([^/?#]*)([^?#]*)(\?[^#]*)?$", value, re.IGNORECASE)
    if not m:
        raise cv.Invalid(
            "base_url must be an absolute https://, http://, wss:// or ws:// URL, "
            "e.g. https://api.openai.com/v1 or http://192.168.1.20:8000/v1"
        )
    host = m.group(2)
    if not host or "@" in host or host.endswith(":"):
        raise cv.Invalid("base_url has no usable host (credentials belong in api_key, not the URL)")
    return m.group(1).lower() in ("https", "wss"), host.lower(), m.group(3), m.group(4) or ""


def _base_url(value):
    value = cv.string_strict(value).strip()
    if any(ord(c) <= 0x20 or ord(c) >= 0x7F or c in '"\\<>' for c in value):
        raise cv.Invalid("base_url contains a space or a character a URL cannot have")
    if len(value) > 159:
        raise cv.Invalid("base_url is limited to 159 characters")
    _split_base_url(value)
    # Trailing slashes on the path are dropped, as the device would.
    path, _, query = value.partition("?")
    return path.rstrip("/") + (("?" + query) if query else "")


def _pattern(regex, what):
    compiled = re.compile(regex)

    def validator(value):
        value = cv.string_strict(value)
        if not compiled.match(value):
            raise cv.Invalid(f"{what} must match {regex}")
        return value

    return validator


_model = _pattern(r"^[A-Za-z0-9._:/-]{1,63}$", "model")
_voice = _pattern(r"^[A-Za-z0-9._:/+-]{1,63}$", "voice")


def _api_key(value):
    value = cv.string(value)
    if len(value) > 255 or any(ord(c) <= 0x20 or ord(c) >= 0x7F or c in '"\\' for c in value):
        raise cv.Invalid("api_key must be at most 255 printable characters without spaces or quotes")
    return value


_PEM_BEGIN = "-----BEGIN CERTIFICATE-----"
_PEM_END = "-----END CERTIFICATE-----"


def _ca_certificate(value):
    """A PEM certificate (the server's own, or the CA that signed it), or empty for none."""
    value = cv.string(value).strip()
    if not value:
        return ""
    if _PEM_BEGIN not in value or _PEM_END not in value:
        raise cv.Invalid(
            f"ca_certificate must be a PEM certificate ({_PEM_BEGIN} ... {_PEM_END})"
        )
    if any(ord(c) >= 0x7F or (ord(c) < 0x20 and c not in "\r\n") for c in value):
        raise cv.Invalid("ca_certificate contains characters a PEM certificate cannot have")
    return value + "\n"


def _certificate_needs_tls(config):
    secure, _, _, _ = _split_base_url(config[CONF_BASE_URL])
    if config[CONF_CA_CERTIFICATE] and not secure:
        raise cv.Invalid(
            "ca_certificate is only used with an https:// or wss:// base_url",
            path=[CONF_CA_CERTIFICATE],
        )
    return config


def _key_required_for_openai(config):
    _, host, _, _ = _split_base_url(config[CONF_BASE_URL])
    if host.split(":")[0] == OPENAI_HOST and not config[CONF_API_KEY]:
        raise cv.Invalid(
            f"api_key is required for {OPENAI_HOST} (a local server may need none)", path=[CONF_API_KEY]
        )
    return config


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
        # The OpenAI SDK's baseURL convention: the device opens <base_url>/realtime?model=<model>.
        cv.Optional(CONF_BASE_URL, default="https://api.openai.com/v1"): _base_url,
        cv.Optional(CONF_API_KEY, default=""): _api_key,
        cv.Optional(CONF_MODEL, default="gpt-realtime-2.1"): _model,
        cv.Optional(CONF_VOICE, default="marin"): _voice,
        cv.Optional(CONF_INSTRUCTIONS, default=""): cv.string,
        cv.Optional(CONF_TURN_DETECTION, default={}): TURN_DETECTION_SCHEMA,
        cv.Optional(CONF_NOISE_REDUCTION, default="none"): cv.one_of(
            "none", "near_field", "far_field", lower=True
        ),
        cv.Optional(CONF_INPUT_TRANSCRIPTION_MODEL, default=""): cv.string,
        cv.Optional(CONF_TOOLS, default=[]): cv.ensure_list(TOOL_SCHEMA),
        cv.Optional(CONF_END_CONVERSATION_TOOL, default=True): cv.boolean,
        # Default mode until a switch (or openai_realtime.set_full_duplex) says otherwise:
        # true = full duplex with barge-in, false = turn-based like the Assist pipeline.
        cv.Optional(CONF_FULL_DUPLEX, default=True): cv.boolean,
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
        # For a wss:// server whose certificate is not from a public authority (self-signed, or a
        # private CA): that certificate in PEM. Without it, only publicly trusted ones are accepted.
        cv.Optional(CONF_CA_CERTIFICATE, default=""): _ca_certificate,
        # false: accept that certificate whatever name or IP address it was issued for.
        cv.Optional(CONF_VERIFY_CERTIFICATE_NAME, default=True): cv.boolean,
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
    _key_required_for_openai,
    _certificate_needs_tls,
    cv.only_on_esp32,
    # One outbound TCP connection per session.
    socket.consume_sockets(1, "openai_realtime"),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic_source = await microphone.microphone_source_to_code(config[CONF_MICROPHONE])
    cg.add(var.set_microphone_source(mic_source))
    spk = await cg.get_variable(config[CONF_SPEAKER])
    cg.add(var.set_speaker(spk))

    cg.add(var.set_base_url(config[CONF_BASE_URL]))
    cg.add(var.set_api_key(config[CONF_API_KEY]))
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_voice(config[CONF_VOICE]))
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
    cg.add(var.set_full_duplex(config[CONF_FULL_DUPLEX]))
    cg.add(var.set_api_version(config[CONF_API_VERSION]))
    cg.add(var.set_idle_timeout(config[CONF_IDLE_TIMEOUT].total_milliseconds))
    cg.add(var.set_max_duration(config[CONF_MAX_DURATION].total_milliseconds))
    cg.add(var.set_playback_latency(config[CONF_PLAYBACK_LATENCY].total_milliseconds))
    cg.add(var.set_playback_buffer_size(config[CONF_PLAYBACK_BUFFER_SIZE]))
    for key, value in config[CONF_EXTRA_HEADERS].items():
        cg.add(var.set_extra_header(key, value))
    if config[CONF_CA_CERTIFICATE]:
        cg.add(var.set_ca_certificate(config[CONF_CA_CERTIFICATE]))
    cg.add(var.set_verify_certificate_name(config[CONF_VERIFY_CERTIFICATE_NAME]))
    # The worker task's stack lives in PSRAM (internal RAM is tight on this board).
    psram.request_external_task_stack()

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

    # esp_websocket_client from the component registry; esp-tls and tcp_transport
    # are built-in IDF components ESPHome excludes unless a component asks for them.
    esp32.add_idf_component(name="espressif/esp_websocket_client", ref="1.6.0")
    # Sends take their own lock instead of the client's: otherwise a send blocked on a server that is
    # not reading (its TCP window closed) also stops the client task from receiving, and the
    # client's PONG replies would go out unlocked, interleaved with our frames.
    esp32.add_idf_sdkconfig_option("CONFIG_ESP_WS_CLIENT_SEPARATE_TX_LOCK", True)
    for name in ("esp-tls", "tcp_transport"):
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
    "openai_realtime.set_full_duplex",
    SetFullDuplexAction,
    OPENAI_REALTIME_ACTION_SCHEMA.extend(
        {cv.Required(CONF_FULL_DUPLEX): cv.templatable(cv.boolean)}
    ),
    synchronous=True,
)
async def openai_realtime_set_full_duplex_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_full_duplex(await cg.templatable(config[CONF_FULL_DUPLEX], args, cg.bool_)))
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
async def openai_realtime_condition_to_code(config, condition_id, template_arg, args):
    var = cg.new_Pvariable(condition_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
