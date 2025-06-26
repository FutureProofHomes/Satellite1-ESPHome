import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL

CONF_CLK_MODE = "i2s_clock_mode"
CONF_SAMPLE_RATE = "sample_rate"
CONF_BITS_PER_SAMPLE = "bits_per_sample"
CONF_PDM = "pdm"
CONF_USE_APLL = "use_apll"
CONF_FIXED_SETTINGS = "fixed_settings"

CONF_MONO = "mono"
CONF_LEFT = "left"
CONF_RIGHT = "right"
CONF_STEREO = "stereo"



INTERNAL_CLK = "internal"
EXTERNAL_CLK = "external"
i2s_mode_t = cg.global_ns.enum("i2s_mode_t")
I2S_CLK_MODES = {
    INTERNAL_CLK: i2s_mode_t.I2S_MODE_MASTER,  # NOLINT
    EXTERNAL_CLK: i2s_mode_t.I2S_MODE_SLAVE,  # NOLINT
}


i2s_dac_mode_t = cg.global_ns.enum("i2s_dac_mode_t")
INTERNAL_DAC_OPTIONS = {
    "left": i2s_dac_mode_t.I2S_DAC_CHANNEL_LEFT_EN,
    "right": i2s_dac_mode_t.I2S_DAC_CHANNEL_RIGHT_EN,
    "stereo": i2s_dac_mode_t.I2S_DAC_CHANNEL_BOTH_EN,
}

i2s_channel_fmt_t = cg.global_ns.enum("i2s_channel_fmt_t")
CHANNEL_FORMAT = {
    # Only load data in left channel (mono mode)
    "left": i2s_channel_fmt_t.I2S_CHANNEL_FMT_ONLY_LEFT,
    # Only load data in right channel (mono mode)
    "right": i2s_channel_fmt_t.I2S_CHANNEL_FMT_ONLY_RIGHT,
    # Separated left and right channel
    "right_left": i2s_channel_fmt_t.I2S_CHANNEL_FMT_RIGHT_LEFT,
    # Load right channel data in both two channels
    "all_right": i2s_channel_fmt_t.I2S_CHANNEL_FMT_ALL_RIGHT,
    # Load left channel data in both two channels
    "all_left": i2s_channel_fmt_t.I2S_CHANNEL_FMT_ALL_LEFT,
}
I2S_CHANNELS = {
    CONF_MONO: i2s_channel_fmt_t.I2S_CHANNEL_FMT_ALL_LEFT,
    CONF_LEFT: i2s_channel_fmt_t.I2S_CHANNEL_FMT_ONLY_LEFT,
    CONF_RIGHT: i2s_channel_fmt_t.I2S_CHANNEL_FMT_ONLY_RIGHT,
    CONF_STEREO: i2s_channel_fmt_t.I2S_CHANNEL_FMT_RIGHT_LEFT,
}

i2s_bits_per_sample_t = cg.global_ns.enum("i2s_bits_per_sample_t")
BITS_PER_SAMPLE = {
    16: i2s_bits_per_sample_t.I2S_BITS_PER_SAMPLE_16BIT,
    24: i2s_bits_per_sample_t.I2S_BITS_PER_SAMPLE_24BIT,
    32: i2s_bits_per_sample_t.I2S_BITS_PER_SAMPLE_32BIT,
}

i2s_bits_per_chan_t = cg.global_ns.enum("i2s_bits_per_chan_t")
BITS_PER_CHANNEL = {
    "default": i2s_bits_per_chan_t.I2S_BITS_PER_CHAN_DEFAULT,
    8: i2s_bits_per_chan_t.I2S_BITS_PER_CHAN_8BIT,
    16: i2s_bits_per_chan_t.I2S_BITS_PER_CHAN_16BIT,
    24: i2s_bits_per_chan_t.I2S_BITS_PER_CHAN_24BIT,
    32: i2s_bits_per_chan_t.I2S_BITS_PER_CHAN_32BIT,
}

_validate_bits = cv.float_with_unit("bits", "bit")

CONFIG_SCHEMA_I2S_COMMON = cv.Schema(
    {
        cv.Optional(CONF_CLK_MODE, default=INTERNAL_CLK): cv.enum(I2S_CLK_MODES),
        cv.Optional(CONF_CHANNEL, default="right_left"): cv.enum(CHANNEL_FORMAT),
        cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_range(min=1),
        cv.Optional(CONF_BITS_PER_SAMPLE, default="32bit"): cv.All(
            _validate_bits, cv.enum(BITS_PER_SAMPLE)
        ),
        cv.Optional(CONF_USE_APLL, default=False): cv.boolean,
        cv.Optional(CONF_FIXED_SETTINGS, default=False): cv.boolean,
    }
)


def get_i2s_config_schema(default_channel, default_rate, default_bits):
    return cv.Schema(
        {
            cv.Optional(CONF_CLK_MODE, default=INTERNAL_CLK): cv.enum(I2S_CLK_MODES),
            cv.Optional(CONF_CHANNEL, default=default_channel): cv.enum(I2S_CHANNELS),
            cv.Optional(CONF_SAMPLE_RATE, default=default_rate): cv.int_range(min=1),
            cv.Optional(CONF_BITS_PER_SAMPLE, default=default_bits): cv.All(
                _validate_bits, cv.enum(BITS_PER_SAMPLE)
            ),
            cv.Optional(CONF_USE_APLL, default=False): cv.boolean,
            cv.Optional(CONF_FIXED_SETTINGS, default=False): cv.boolean,
        }
    )
