import gzip
import hashlib
import logging
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    button,
    event,
    number,
    select,
    sensor,
    switch,
    text,
    text_sensor,
    update,
)
from esphome.components.light.types import LightState
from esphome.const import CONF_ID, Framework
from esphome.core import HexInt
import esphome.final_validate as fv

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@FutureProofHomes"]
DEPENDENCIES = ["network"]

# Auto-loaded rather than declared a dependency so that the missing-web_server case is reported by
# _final_validate below, which can say what to add, instead of by the generic dependency error.
# web_server auto-loads this too; both resolve to the same single instance.
AUTO_LOAD = ["web_server_base"]
MULTI_CONF = False

# No consume_sockets() call, deliberately. Raising it inflates CONFIG_LWIP_MAX_SOCKETS without
# lifting esp_http_server's own ceiling of max_open_sockets = 7, which is the number that actually
# bounds concurrent tabs. The app adds no listener of its own: it is a handler on the socket
# web_server_base already reserves.

CONF_INDEX_ID = "index_id"
CONF_ENTITIES = "entities"

satellite1_web_ui_ns = cg.esphome_ns.namespace("satellite1_web_ui")
Satellite1WebUI = satellite1_web_ui_ns.class_("Satellite1WebUI", cg.Component)

# The frontend addresses entities by a stable logical key; the device resolves each key to the
# "<domain>/<name>" that web_server actually answers to, at runtime, from get_name().
#
# This indirection is not decoration. web_server identifies entities by *display name*, in both
# directions: set_json_id builds the /events id as "{domain}/{name}" (web_server.cpp:550, "Uses
# names (not object_id) to avoid UTF-8 collision issues") and UrlMatch::match_entity compares
# `this->id == entity->get_name()` (web_server.cpp:167). So the alternative is ~25 display names
# hardcoded in JavaScript, where a YAML rename becomes a silently broken control with nothing to
# catch it - and Phase 4 exists to change entity dispositions.
#
# One entity proves the point on its own. `update:` in satellite1.yaml is declared `name: None`,
# which ESPHome renders as the device name, so on the test device the Install button lives at
# "update/Satellite1 c5ac00" - a name that differs on every device ever built. There is no string
# the frontend could have contained.
#
# Radar entities are deliberately absent. satellite1_radar registers "Radar Target", "Radar
# Detected" and friends at runtime from constants in its own C++, so those names are owned by code
# rather than by anyone's YAML, and there is no id to point at here anyway.
_ENTITY_DOMAINS = {
    "sensors": ("sensor", sensor.Sensor),
    "binary_sensors": ("binary_sensor", binary_sensor.BinarySensor),
    "text_sensors": ("text_sensor", text_sensor.TextSensor),
    "switches": ("switch", switch.Switch),
    "numbers": ("number", number.Number),
    "selects": ("select", select.Select),
    "buttons": ("button", button.Button),
    "texts": ("text", text.Text),
    "events": ("event", event.Event),
    "updates": ("update", update.UpdateEntity),
    "lights": ("light", LightState),
}

_ENTITIES_SCHEMA = cv.Schema(
    {
        cv.Optional(group): cv.Schema({cv.string_strict: cv.use_id(entity_class)})
        for group, (_, entity_class) in _ENTITY_DOMAINS.items()
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Satellite1WebUI),
            cv.GenerateID(CONF_INDEX_ID): cv.declare_id(cg.uint8),
            cv.Optional(CONF_ENTITIES, default={}): _ENTITIES_SCHEMA,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_framework(Framework.ESP_IDF),
)

_DIST = Path(__file__).parent / "dist" / "index.html"


def _final_validate(config):
    # The app is a handler on the shared server, so web_server is a hard requirement rather than
    # something to work around. Without it global_web_server_base is never constructed and setup()
    # would dereference null - a boot loop, not a missing page. Caught here so it is a config error
    # with a fix in it instead.
    full = fv.full_config.get()
    web_server = full.get("web_server")
    if web_server is None:
        raise cv.Invalid(
            "satellite1_web_ui serves the web app from ESPHome's shared web server, so a "
            "top-level 'web_server:' block is required. Add one on port 80."
        )

    # The whole point of the app is to show controls the Home Assistant UI deliberately does not,
    # and those entities are internal: true. Without include_internal they never reach /events and
    # the app renders a page with most of its controls missing - a confusing runtime failure that
    # is worth turning into a config error.
    if not web_server.get("include_internal"):
        raise cv.Invalid(
            "satellite1_web_ui needs the entities it renders to reach /events, so "
            "'web_server:' must set 'include_internal: true'."
        )

    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Sorted so the generated code, and the JSON the device serves, are stable across builds
    # regardless of how the YAML happens to be ordered.
    for group in sorted(config[CONF_ENTITIES]):
        domain, _ = _ENTITY_DOMAINS[group]
        for key in sorted(config[CONF_ENTITIES][group]):
            entity = await cg.get_variable(config[CONF_ENTITIES][group][key])
            cg.add(var.add_entity(key, domain, entity))

    if not _DIST.is_file():
        raise cv.Invalid(
            f"The web UI bundle is missing at {_DIST}. Build it with "
            "'npm ci && npm run build' in esphome/components/satellite1_web_ui/frontend."
        )

    # Same gzip.compress -> cg.progmem_array path the radar tuner pages already use. dist/ holds
    # the uncompressed document so that the CI drift check diffs readable text; the compression
    # happens here, where it costs nothing at runtime.
    html = _DIST.read_bytes()
    gz = gzip.compress(html, compresslevel=9)

    index = cg.progmem_array(config[CONF_INDEX_ID], tuple(map(HexInt, gz)))
    cg.add(var.set_index(index, len(gz)))

    # Hashed over the uncompressed source, so the tag does not move when zlib's output does. It is
    # emitted as a string literal, which is what ESP-IDF's httpd_resp_set_hdr requires: it stores
    # the pointer rather than copying, so the value has to outlive the response.
    cg.add(var.set_etag(f'"{hashlib.sha256(html).hexdigest()[:16]}"'))

    _LOGGER.info(
        "Satellite1 web UI bundle: %d bytes raw, %d bytes gzipped", len(html), len(gz)
    )
