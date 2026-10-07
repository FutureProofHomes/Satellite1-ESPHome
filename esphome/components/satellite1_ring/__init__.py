"""Satellite1 LED ring styles: the stored per-moment animations the Styled effect draws.

See ring_fx.h for how the two partition lights share the ring, and config/common/led_ring.yaml for
the wiring: control_leds calls set_moment() and starts the Styled effect, and on_preview re-runs
control_leds when a web preview starts or ends.
"""

from esphome import automation
import esphome.codegen as cg
from esphome.components import light, select, text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

CODEOWNERS = ["@FutureProofHomes"]
DEPENDENCIES = ["light"]
AUTO_LOAD = ["select", "text_sensor"]

CONF_LIGHT_ID = "light_id"
CONF_TIMER_RATIO = "timer_ratio"
CONF_VOLUME = "volume"
CONF_MIC_MUTED = "mic_muted"
CONF_SPEAKER_SILENT = "speaker_silent"
CONF_STYLE_SELECT = "style_select"
CONF_MOMENT_SENSOR = "moment_sensor"
CONF_ON_PREVIEW = "on_preview"

# The select's options, in the order ring_fx_core.h numbers the presets; Custom is the edited copy.
STYLE_OPTIONS = ["Classic", "Calm", "Aurora", "Party", "Minimal", "Custom"]

satellite1_ring_ns = cg.esphome_ns.namespace("satellite1_ring")
RingFx = satellite1_ring_ns.class_("RingFx", cg.Component)
RingStyleSelect = satellite1_ring_ns.class_(
    "RingStyleSelect", select.Select, cg.Parented.template(RingFx)
)
PreviewTrigger = satellite1_ring_ns.class_("PreviewTrigger", automation.Trigger.template())

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RingFx),
        # The customer's light, whose color and brightness every ring-colored moment follows.
        cv.Required(CONF_LIGHT_ID): cv.use_id(light.LightState),
        # Inputs the data-driven moments draw, from whichever entities mean them on this build.
        cv.Optional(CONF_TIMER_RATIO): cv.returning_lambda,
        cv.Optional(CONF_VOLUME): cv.returning_lambda,
        cv.Optional(CONF_MIC_MUTED): cv.returning_lambda,
        cv.Optional(CONF_SPEAKER_SILENT): cv.returning_lambda,
        cv.Optional(CONF_STYLE_SELECT): select.select_schema(
            RingStyleSelect, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:palette"
        ),
        # Which moment the ring is showing, for the web UI's live ring (over /events) and for
        # automations. Changes a few times per conversation, never per frame.
        cv.Optional(CONF_MOMENT_SENSOR): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:dots-circle"
        ),
        cv.Optional(CONF_ON_PREVIEW): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(PreviewTrigger)}
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_light(await cg.get_variable(config[CONF_LIGHT_ID])))

    for key, setter, kind in (
        (CONF_TIMER_RATIO, "set_timer_ratio", cg.float_),
        (CONF_VOLUME, "set_volume", cg.float_),
        (CONF_MIC_MUTED, "set_mic_muted", cg.bool_),
        (CONF_SPEAKER_SILENT, "set_speaker_silent", cg.bool_),
    ):
        if key in config:
            lam = await cg.process_lambda(config[key], [], return_type=kind)
            cg.add(getattr(var, setter)(lam))

    if conf := config.get(CONF_STYLE_SELECT):
        sel = await select.new_select(conf, options=STYLE_OPTIONS)
        await cg.register_parented(sel, var)
        cg.add(var.set_style_select(sel))

    if conf := config.get(CONF_MOMENT_SENSOR):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_moment_sensor(sens))

    for conf in config.get(CONF_ON_PREVIEW, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
