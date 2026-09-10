import gzip
import hashlib
import logging
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
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

satellite1_web_ui_ns = cg.esphome_ns.namespace("satellite1_web_ui")
Satellite1WebUI = satellite1_web_ui_ns.class_("Satellite1WebUI", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Satellite1WebUI),
            cv.GenerateID(CONF_INDEX_ID): cv.declare_id(cg.uint8),
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
