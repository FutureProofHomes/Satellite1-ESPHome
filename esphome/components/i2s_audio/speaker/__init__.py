import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_CHANNEL,
    CONF_ID,
    CONF_NUM_CHANNELS, 
    CONF_TIMEOUT
)
from esphome.components import esp32, speaker

from .. import i2s_settings as i2s

from .. import (
    CONF_I2S_AUDIO_ID,
    CONF_I2S_DOUT_PIN,
    CONF_LEFT,
    CONF_MONO,
    CONF_RIGHT,
    CONF_STEREO,
    I2SAudioComponent,
    i2s_audio_component_schema,
    I2SWriter,
    i2s_audio_ns,
    register_i2s_writer,
)

CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["i2s_audio"]

I2SAudioSpeaker = i2s_audio_ns.class_(
    "I2SAudioSpeaker", cg.Component, speaker.Speaker, I2SWriter
)
CONF_BUFFER_DURATION = "buffer_duration"
CONF_NEVER = "never"
i2s_dac_mode_t = cg.global_ns.enum("i2s_dac_mode_t")

CONF_MUTE_PIN = "mute_pin"
CONF_DAC_TYPE = "dac_type"

INTERNAL_DAC_OPTIONS = {
    "left": i2s_dac_mode_t.I2S_DAC_CHANNEL_LEFT_EN,
    "right": i2s_dac_mode_t.I2S_DAC_CHANNEL_RIGHT_EN,
    "stereo": i2s_dac_mode_t.I2S_DAC_CHANNEL_BOTH_EN,
}

NO_INTERNAL_DAC_VARIANTS = [esp32.const.VARIANT_ESP32S2]


def validate_esp32_variant(config):
    if config[CONF_DAC_TYPE] != "internal":
        return config
    variant = esp32.get_esp32_variant()
    if variant in NO_INTERNAL_DAC_VARIANTS:
        raise cv.Invalid(f"{variant} does not have an internal DAC")
    return config


BASE_SCHEMA = (
    speaker.SPEAKER_SCHEMA.extend(
        i2s_audio_component_schema(
            I2SAudioSpeaker,
            default_sample_rate=16000,
            default_channel=CONF_MONO,
            default_bits_per_sample="16bit",
        )
    )
    .extend(
        {
            cv.Optional(
                CONF_BUFFER_DURATION, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TIMEOUT, default="500ms"): cv.Any(
                cv.positive_time_period_milliseconds,
                cv.one_of(CONF_NEVER, lower=True),
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


def _set_num_channels_from_config(config):
    if config[CONF_CHANNEL] in (CONF_MONO, CONF_LEFT, CONF_RIGHT):
        config[CONF_NUM_CHANNELS] = 1
    else:
        config[CONF_NUM_CHANNELS] = 2

    return config



CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            "external": speaker.SPEAKER_SCHEMA.extend(
                {
                    cv.GenerateID(): cv.declare_id(I2SAudioSpeaker),
                    cv.GenerateID(CONF_I2S_AUDIO_ID): cv.use_id(I2SAudioComponent),
                    cv.Required(
                        CONF_I2S_DOUT_PIN
                    ): pins.internal_gpio_output_pin_number,
                    cv.Optional(
                CONF_BUFFER_DURATION, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TIMEOUT, default="500ms"): cv.Any(
                cv.positive_time_period_milliseconds,
                cv.one_of(CONF_NEVER, lower=True),
            ),
                }
            )
            .extend(
                i2s.get_i2s_config_schema(
                    default_channel="right_left", default_rate=48000, default_bits="32bit"
                )
            )
            .extend(cv.COMPONENT_SCHEMA),
        },
        key=CONF_DAC_TYPE,
    ),
    validate_esp32_variant,
    _set_num_channels_from_config
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)

    await cg.register_parented(var, config[CONF_I2S_AUDIO_ID])

    await register_i2s_writer(var, config)
    if config[CONF_TIMEOUT] != CONF_NEVER:
        cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    cg.add(var.set_buffer_duration(config[CONF_BUFFER_DURATION]))
