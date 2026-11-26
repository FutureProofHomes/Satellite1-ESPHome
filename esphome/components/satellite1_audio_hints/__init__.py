import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.i2s_audio.speaker import I2SAudioSpeaker
from esphome.components.satellite1.satellite1 import Satellite1
from esphome.const import CONF_ID

DEPENDENCIES = ["satellite1", "i2s_audio"]
namespace = cg.esphome_ns.namespace("satellite1_audio_hints")
AudioHints = namespace.class_("AudioHints", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(AudioHints),
    cv.Required("satellite1_id"): cv.use_id(Satellite1),
    cv.Required("speaker_id"): cv.use_id(I2SAudioSpeaker),
    cv.Required("voice_active"): cv.returning_lambda,
    cv.Optional("enabled", default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    parent = await cg.get_variable(config["satellite1_id"])
    speaker = await cg.get_variable(config["speaker_id"])
    voice_active = await cg.process_lambda(config["voice_active"], [], return_type=cg.bool_)
    cg.add(var.set_satellite(parent))
    cg.add(var.set_speaker(speaker))
    cg.add(var.set_voice_active(voice_active))
    cg.add(var.set_enabled(config["enabled"]))
