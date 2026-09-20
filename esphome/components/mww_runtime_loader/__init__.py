"""Runtime wake word loading: the device downloads microWakeWord models over HTTP and runs them.

The component keeps two wake word "slots" - the product's simultaneous-word ceiling - each holding
a built-in model id or a manifest URL. Slot writes arrive from satellite1_web_ui's endpoints; this
component owns the download, the validation ladder, micro_wake_word's runtime model API, and the
persistence (the URL is remembered, the model is re-downloaded at boot - never stored in flash).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import logger
from esphome.components.http_request import HttpRequestComponent
from esphome.components.micro_wake_word import MicroWakeWord
from esphome.const import CONF_ID

CODEOWNERS = ["@FutureProofHomes"]
DEPENDENCIES = ["micro_wake_word", "http_request", "network", "logger"]
# json parses the downloaded manifest; auto-loaded so no config has to know that detail.
AUTO_LOAD = ["json"]
MULTI_CONF = False

CONF_MICRO_WAKE_WORD_ID = "micro_wake_word_id"
CONF_HTTP_REQUEST_ID = "http_request_id"

mww_runtime_loader_ns = cg.esphome_ns.namespace("mww_runtime_loader")
MwwRuntimeLoader = mww_runtime_loader_ns.class_("MwwRuntimeLoader", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MwwRuntimeLoader),
        cv.GenerateID(CONF_MICRO_WAKE_WORD_ID): cv.use_id(MicroWakeWord),
        cv.GenerateID(CONF_HTTP_REQUEST_ID): cv.use_id(HttpRequestComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # The Wake Word Tuner's score channel: micro_wake_word logs each detection's probabilities, and
    # this compiles in the logger's listener vector so the component can subscribe to those lines.
    logger.request_log_listener()

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_micro_wake_word(await cg.get_variable(config[CONF_MICRO_WAKE_WORD_ID])))
    cg.add(var.set_http_request(await cg.get_variable(config[CONF_HTTP_REQUEST_ID])))
    # satellite1_web_ui compiles its slot endpoints only when this component is in the build.
    cg.add_define("USE_SAT1_MWW_LOADER", True)
