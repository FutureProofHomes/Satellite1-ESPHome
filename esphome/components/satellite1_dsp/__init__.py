"""Satellite1 speaker EQ: the per-output profiles and presets the XMOS speaker EQ plays.

See satellite1_dsp.h for when profiles reach the XMOS and how support is detected, and
config/common/dsp.yaml for the wiring.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.satellite1.audio_dac import DACProxy
from esphome.components.satellite1.satellite1 import CONF_SATELLITE1, Satellite1
from esphome.const import CONF_ID

CODEOWNERS = ["@FutureProofHomes"]
DEPENDENCIES = ["satellite1"]

CONF_DAC_PROXY_ID = "dac_proxy_id"

satellite1_dsp_ns = cg.esphome_ns.namespace("satellite1_dsp")
SpeakerDsp = satellite1_dsp_ns.class_("SpeakerDsp", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SpeakerDsp),
        cv.GenerateID(CONF_SATELLITE1): cv.use_id(Satellite1),
        # Which output is live, so the line-out profile plays while the jack is in. Without it the
        # built-in speaker's profile always plays.
        cv.Optional(CONF_DAC_PROXY_ID): cv.use_id(DACProxy),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_satellite1(await cg.get_variable(config[CONF_SATELLITE1])))
    if CONF_DAC_PROXY_ID in config:
        cg.add_define("USE_SAT1_DSP_DAC_PROXY")
        cg.add(var.set_dac_proxy(await cg.get_variable(config[CONF_DAC_PROXY_ID])))
