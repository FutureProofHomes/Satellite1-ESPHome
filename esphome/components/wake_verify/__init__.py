"""Second-stage wake word check: the last moments of microphone audio go to Home Assistant's
speech-to-text API, and the transcript is matched against the wake phrases.

The component keeps a short rolling buffer of the same microphone channel micro_wake_word listens
to. ``wake_verify.check`` hands it a wake, and ``on_wake`` fires with the wake word to start the
voice assistant with:

- ``off``: on_wake fires at once with the detected word; nothing is checked, no audio is buffered,
  and ``ha_url``, ``ha_token`` and ``stt_engine`` may be left out.
- ``log``: on_wake fires at once with the detected word, and the check runs in the background and is
  only logged, so the logs show what ``reroute`` would have done.
- ``reroute``: on_wake waits for the check. A transcript matching the detected word passes it on; one
  matching a different enabled wake word passes that word instead (the model fired on the wrong
  phrase); one matching none drops the wake. If speech-to-text fails or takes longer than
  ``decision_timeout``, the detected word goes ahead, so a server problem never makes the device deaf.

Words in ``skip_wake_words`` always pass straight through and are never offered as reroute targets.
A wake that arrives while another is waiting for its verdict (two models firing on the same speech)
joins that check rather than starting its own. Quiet clips are boosted before they are sent.
"""

from esphome import automation
import esphome.codegen as cg
from esphome.components import micro_wake_word, microphone
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MICROPHONE, CONF_MODE, CONF_TIMEOUT

AUTO_LOAD = ["json"]
DEPENDENCIES = ["esp32", "microphone", "micro_wake_word", "psram"]
CODEOWNERS = ["@wsieber"]

CONF_HA_URL = "ha_url"
CONF_HA_TOKEN = "ha_token"
CONF_STT_ENGINE = "stt_engine"
CONF_LANGUAGE = "language"
CONF_BUFFER_DURATION = "buffer_duration"
CONF_POST_ROLL = "post_roll"
CONF_DECISION_TIMEOUT = "decision_timeout"
CONF_MATCH_THRESHOLD = "match_threshold"
CONF_SKIP_WAKE_WORDS = "skip_wake_words"
CONF_MICRO_WAKE_WORD = "micro_wake_word"
CONF_ON_WAKE = "on_wake"
CONF_WAKE_WORD = "wake_word"

wake_verify_ns = cg.esphome_ns.namespace("wake_verify")
WakeVerify = wake_verify_ns.class_("WakeVerify", cg.Component)
CheckAction = wake_verify_ns.class_("CheckAction", automation.Action)

MODES = {"off": 0, "log": 1, "reroute": 2}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WakeVerify),
        cv.Required(CONF_MICROPHONE): microphone.microphone_source_schema(),
        # Reroute targets are the micro_wake_word models enabled at the time of the wake.
        cv.Required(CONF_MICRO_WAKE_WORD): cv.use_id(micro_wake_word.MicroWakeWord),
        cv.Optional(CONF_MODE, default="log"): cv.one_of(*MODES, lower=True),
        # Needed unless the mode is off (see _require_server below).
        cv.Optional(CONF_HA_URL): cv.Any(cv.one_of(""), cv.url),
        cv.Optional(CONF_HA_TOKEN, default=""): cv.string,
        cv.Optional(CONF_STT_ENGINE, default=""): cv.string,
        cv.Optional(CONF_LANGUAGE, default="en"): cv.string_strict,
        cv.Optional(CONF_BUFFER_DURATION, default="1800ms"): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(min=cv.TimePeriod(milliseconds=500), max=cv.TimePeriod(seconds=4)),
        ),
        # Detection lands a little before the word's tail has been captured; wait this long first.
        cv.Optional(CONF_POST_ROLL, default="200ms"): cv.positive_time_period_milliseconds,
        # The HTTP request's own limit; in log mode it is the only one.
        cv.Optional(CONF_TIMEOUT, default="3s"): cv.positive_time_period_milliseconds,
        # In reroute mode, how long the wake waits for a verdict before going ahead unchecked.
        cv.Optional(CONF_DECISION_TIMEOUT, default="1500ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_MATCH_THRESHOLD, default=0.75): cv.percentage,
        cv.Optional(CONF_SKIP_WAKE_WORDS, default=[]): cv.ensure_list(cv.string),
        cv.Required(CONF_ON_WAKE): automation.validate_automation(single=True),
    }
).extend(cv.COMPONENT_SCHEMA)


def _require_server(config):
    if config[CONF_MODE] == "off":
        return config
    for key in (CONF_HA_URL, CONF_HA_TOKEN, CONF_STT_ENGINE):
        if not config.get(key):
            raise cv.Invalid(f"'{key}' is required when mode is '{config[CONF_MODE]}'", path=[key])
    return config


CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, _require_server)

FINAL_VALIDATE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_MICROPHONE): microphone.final_validate_microphone_source_schema(
            "wake_verify", sample_rate=16000
        ),
    },
    extra=cv.ALLOW_EXTRA,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Passive: micro_wake_word owns starting and stopping the microphone.
    mic_source = await microphone.microphone_source_to_code(config[CONF_MICROPHONE], passive=True)
    cg.add(var.set_microphone_source(mic_source))
    mww = await cg.get_variable(config[CONF_MICRO_WAKE_WORD])
    cg.add(var.set_micro_wake_word(mww))

    cg.add(var.set_mode(MODES[config[CONF_MODE]]))
    cg.add(var.set_ha_url(config.get(CONF_HA_URL, "").rstrip("/")))
    cg.add(var.set_ha_token(config[CONF_HA_TOKEN]))
    cg.add(var.set_stt_engine(config[CONF_STT_ENGINE]))
    cg.add(var.set_language(config[CONF_LANGUAGE]))
    cg.add(var.set_buffer_duration_ms(config[CONF_BUFFER_DURATION].total_milliseconds))
    cg.add(var.set_post_roll_ms(config[CONF_POST_ROLL].total_milliseconds))
    cg.add(var.set_timeout_ms(config[CONF_TIMEOUT].total_milliseconds))
    cg.add(var.set_decision_timeout_ms(config[CONF_DECISION_TIMEOUT].total_milliseconds))
    cg.add(var.set_match_threshold(config[CONF_MATCH_THRESHOLD]))
    for word in config[CONF_SKIP_WAKE_WORDS]:
        cg.add(var.add_skip_wake_word(word))

    await automation.build_automation(
        var.get_wake_trigger(), [(cg.std_string, "wake_word")], config[CONF_ON_WAKE]
    )


@automation.register_action(
    "wake_verify.check",
    CheckAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WakeVerify),
            cv.Required(CONF_WAKE_WORD): cv.templatable(cv.string),
        }
    ),
    synchronous=True,
)
async def check_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    wake_word = await cg.templatable(config[CONF_WAKE_WORD], args, cg.std_string)
    cg.add(var.set_wake_word(wake_word))
    return var
