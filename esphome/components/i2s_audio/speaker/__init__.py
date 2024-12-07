import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID, CONF_TIMEOUT
from esphome.components import esp32, speaker

from .. import i2s_settings as i2s

from .. import (
    CONF_I2S_AUDIO_ID,
    CONF_I2S_DOUT_PIN,
    I2SAudioComponent,
    I2SWriter,
    i2s_audio_ns,
    register_i2s_writer,
)

CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["i2s_audio"]

I2SAudioSpeaker = i2s_audio_ns.class_(
    "I2SAudioSpeaker", cg.Component, speaker.Speaker, I2SWriter
)

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
                        CONF_TIMEOUT, default="500ms"
                    ): cv.positive_time_period_milliseconds,
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
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)

    await cg.register_parented(var, config[CONF_I2S_AUDIO_ID])

    await register_i2s_writer(var, config)
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
