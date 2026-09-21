import gzip
import hashlib
import json
import logging
from pathlib import Path

import esphome.automation as automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    audio,
    binary_sensor,
    button,
    event,
    media_player,
    number,
    select,
    sensor,
    switch,
    text,
    text_sensor,
    update,
)
from esphome.components.light.types import LightState
from esphome.components.micro_wake_word import MicroWakeWord

# Optional on purpose: this component must keep validating for a build that does not carry the
# runtime wake word loader, so the import failing simply removes the option.
try:
    from esphome.components.mww_runtime_loader import MwwRuntimeLoader
except ImportError:
    MwwRuntimeLoader = None

# Optional for the same reason: without crash_report in the build the crash endpoints simply do
# not exist, and the app's Diagnostics route renders without the card.
try:
    from esphome.components.crash_report import CrashReport
except ImportError:
    CrashReport = None
from esphome.components.sendspin import (
    SendspinHub,
    request_controller_support,
    request_metadata_support,
)
from esphome.components.voice_assistant import VoiceAssistant
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID, WebServerBase
from esphome.const import CONF_ID, Framework
from esphome.core import CORE, HexInt
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
CONF_NO_SENSOR_ID = "no_sensor_id"
CONF_MANIFEST_ID = "manifest_id"
CONF_ICON_192_ID = "icon_192_id"
CONF_ICON_512_ID = "icon_512_id"
CONF_ICON_180_ID = "icon_180_id"
CONF_ENTITIES = "entities"
CONF_LOGIN_MIC_AVAILABLE = "login_mic_available"
CONF_ON_LOGIN_WINDOW = "on_login_window"
CONF_ON_LOGIN_WINDOW_END = "on_login_window_end"
CONF_MICRO_WAKE_WORD_ID = "micro_wake_word_id"
CONF_WAKE_LOADER_ID = "wake_loader_id"
CONF_CRASH_REPORT_ID = "crash_report_id"
CONF_VOICE_ASSISTANT_ID = "voice_assistant_id"
CONF_VOICE_PHASE = "voice_phase"
CONF_MEDIA_PLAYER_ID = "media_player_id"
CONF_SENDSPIN_MEDIA_PLAYER_ID = "sendspin_media_player_id"
CONF_SENDSPIN_HUB_ID = "sendspin_hub_id"
CONF_ON_HA_REFRESH = "on_ha_refresh"
CONF_ON_HA_SELECT = "on_ha_select"
CONF_ON_MA_REFRESH = "on_ma_refresh"
CONF_ON_MA_LIKE = "on_ma_like"
CONF_ON_MA_JOIN = "on_ma_join"
CONF_ON_MA_UNJOIN = "on_ma_unjoin"
CONF_ON_MA_VOLUME = "on_ma_volume"
CONF_ON_MA_SEEK = "on_ma_seek"
CONF_ON_SELECTION_CHANGE = "on_selection_change"
CONF_SOUNDS = "sounds"

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

def _request_sendspin_roles(config):
    # Runs at validation time, which is when sendspin's own platforms request their roles - the
    # flags live in CORE.data and sendspin's to_code reads them, so a request made from our own
    # to_code could land after sendspin has already generated its role config. The metadata and
    # controller roles are what the hub callbacks below ride, and neither is guaranteed on by
    # anything else: metadata happens to be requested by the title/artist text sensors and the
    # controller by the media_player platform, but this component must not depend on which other
    # platforms a config declares. Requesting them is idempotent.
    if CONF_SENDSPIN_HUB_ID in config:
        request_metadata_support()
        request_controller_support()
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Satellite1WebUI),
            cv.GenerateID(CONF_INDEX_ID): cv.declare_id(cg.uint8),
            cv.GenerateID(CONF_NO_SENSOR_ID): cv.declare_id(cg.uint8),
            cv.GenerateID(CONF_MANIFEST_ID): cv.declare_id(cg.uint8),
            cv.GenerateID(CONF_ICON_192_ID): cv.declare_id(cg.uint8),
            cv.GenerateID(CONF_ICON_512_ID): cv.declare_id(cg.uint8),
            cv.GenerateID(CONF_ICON_180_ID): cv.declare_id(cg.uint8),
            # The shared server the session gate registers on. Resolved with use_id so codegen
            # orders web_server_base's construction ahead of the registration statement.
            cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(WebServerBase),
            cv.Optional(CONF_ENTITIES, default={}): _ENTITIES_SCHEMA,
            # Whether the microphones could hear a voice approval right now, read when a pairing
            # window opens. From YAML because only YAML knows which entities mean "muted" on this
            # build; without it every window opens button-only.
            cv.Optional(CONF_LOGIN_MIC_AVAILABLE): cv.returning_lambda,
            # The pairing window's lifecycle, for the announcement, the LED breathe and the model
            # arming. Open fires with (mode, secret); end fires with (result). Both optional so a
            # build without the YAML wiring still compiles - windows then open and close silently,
            # approvable by the button alone.
            cv.Optional(CONF_ON_LOGIN_WINDOW): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_LOGIN_WINDOW_END): automation.validate_automation(single=True),
            # Timers and the assistant's phase have no entity to read them from: get_timers() is a
            # plain vector on the component, and the phase is a `globals:` int that config/ already
            # maintains through the existing on_listening / on_stt_vad_* triggers. Both optional so
            # that a build without voice_assistant still compiles.
            cv.Optional(CONF_VOICE_ASSISTANT_ID): cv.use_id(VoiceAssistant),
            cv.Optional(CONF_VOICE_PHASE): cv.returning_lambda,
            # Wake words are not entities. micro_wake_word creates no switch and no select, so the
            # models never reach /events or the entity REST API, and the component itself is the only
            # way to see or change which are armed. Optional for the same reason as the two above.
            cv.Optional(CONF_MICRO_WAKE_WORD_ID): cv.use_id(MicroWakeWord),
            # The runtime wake word loader, which owns the two slots the wake words endpoint speaks
            # when it is present - and the download/validate/persist machinery behind them. Optional
            # so a build without it keeps the old per-index toggle endpoint.
            **(
                {cv.Optional(CONF_WAKE_LOADER_ID): cv.use_id(MwwRuntimeLoader)}
                if MwwRuntimeLoader is not None
                else {}
            ),
            # The crash flight recorder, which owns the records, the pre-crash log tail and the
            # core dump image the /api/sat1/crash* endpoints serve. Optional so a build without it
            # keeps compiling; the endpoints then do not exist at all (the define comes from
            # crash_report's own to_code).
            **(
                {cv.Optional(CONF_CRASH_REPORT_ID): cv.use_id(CrashReport)}
                if CrashReport is not None
                else {}
            ),
            # Media players are not covered by web_server either - it registers no media_player
            # handler, so they ride neither /events nor the entity REST API. These two back
            # GET/POST /api/sat1/media: the local speaker player, and the Sendspin group player the
            # endpoints prefer while it is playing. Both optional so a build with neither still
            # compiles; the endpoints answer 404 without them.
            cv.Optional(CONF_MEDIA_PLAYER_ID): cv.use_id(media_player.MediaPlayer),
            cv.Optional(CONF_SENDSPIN_MEDIA_PLAYER_ID): cv.use_id(media_player.MediaPlayer),
            # The Sendspin hub itself, beyond the media_player entity above. The protocol carries
            # far more than the entity model can express - track metadata with an artwork URL,
            # controller state with shuffle/repeat and the server's supported-command list, and an
            # interpolated track position - and the hub exposes each through public callbacks or
            # getters. With this set, GET /api/sat1/media grows those fields; without it the
            # endpoint serves exactly what it always did. The media footer is the only consumer.
            cv.Optional(CONF_SENDSPIN_HUB_ID): cv.use_id(SendspinHub),
            # Fired when a browser posts to /api/sat1/ha/refresh. The work is a Home Assistant action
            # call, which belongs in YAML next to the rest of the ladder, so the component only says
            # that someone asked. Optional: without common/web_ui_ha.yaml the endpoint accepts the
            # request and nothing listens, which is the honest behaviour for a build with no data layer.
            cv.Optional(CONF_ON_HA_REFRESH): automation.validate_automation(single=True),
            # Fired with `entity` and `option` when a browser posts to /api/sat1/ha/select, which is how
            # the app changes which assistant answers which wake word. That mapping is four select
            # entities Home Assistant creates for this device and keeps on its own side, so unlike every
            # other control in the app there is no local state to write - only an action to call, and
            # calling it belongs in YAML beside the ladder.
            cv.Optional(CONF_ON_HA_SELECT): automation.validate_automation(single=True),
            # The Music Assistant relay: fired from loop() for POST /api/sat1/ma/refresh and
            # /api/sat1/ma/<cmd>, each implemented in common/web_ui_media.yaml as one
            # homeassistant.action call - the same split as the two above, for the same reason. All
            # optional: without that file the endpoints queue into silence, which is the honest
            # behaviour for a build with no Music Assistant half.
            cv.Optional(CONF_ON_MA_REFRESH): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_MA_LIKE): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_MA_JOIN): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_MA_UNJOIN): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_MA_VOLUME): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_MA_SEEK): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_SELECTION_CHANGE): automation.validate_automation(single=True),
            # Sounds served at GET /api/sat1/sounds/<name>, session-gate exempt, for the routing
            # feature's third-party targets: a Sonos playing the mirrored timer ring cannot read
            # this device's flash, so it fetches the same audio_file bytes over HTTP instead. The
            # key is the URL tail and should carry the real extension ("timer_finished.mp3") -
            # remote decoders key on it - while the Content-Type is derived from the embedded
            # bytes at request time, so the two cannot drift apart.
            cv.Optional(CONF_SOUNDS, default={}): cv.Schema(
                {cv.string_strict: cv.use_id(audio.AudioFile)}
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_framework(Framework.ESP_IDF),
    _request_sendspin_roles,
)

_DIST = Path(__file__).parent / "dist" / "index.html"

# The Presence route's no-sensor product photo, pre-optimized (WebP q70, from the PNG kept at
# frontend/assets/no-sensor-source.png for provenance). Committed as the encoded artifact rather than
# encoded at codegen, because Pillow's WebP output is not guaranteed byte-stable across versions and
# the CI drift check needs the shipped bytes to be the reviewed bytes.
_NO_SENSOR = Path(__file__).parent / "assets" / "no_sensor.webp"


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

    # The session gate owns authentication now: cookie sessions for browsers, with the same digest
    # check as the curl/script fallback. A web_server auth: block would install credentials on
    # web_server_base, which wraps every later handler in AuthMiddlewareHandler - putting a digest
    # prompt behind the gate's cookie check and breaking every signed-in browser. Refused here,
    # where the error can say what to do instead.
    if web_server.get("auth") is not None:
        raise cv.Invalid(
            "satellite1_web_ui provides cookie-session authentication with a digest fallback, "
            "so 'web_server:' must not carry an 'auth:' block. Hand the credentials to "
            "satellite1_web_ui via set_credentials() at boot instead - see "
            "config/common/web_ui.yaml."
        )

    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # The session gate registers from generated code, not from setup(). Codegen statements all run
    # before any component's setup(), so the gate lands at position 0 of web_server_base's handler
    # vector - ahead of satellite1_radar (which registers at setup priority 800), ahead of this
    # component's own handler (250), ahead of web_server (249). First position is what makes it a
    # gate, and init() replays the vector in order, so it survives the Web UI switch's listener
    # restarts. The get_variable is what orders web_server_base's construction ahead of this.
    await cg.get_variable(config[CONF_WEB_SERVER_BASE_ID])
    cg.add(var.register_session_gate())

    # The gate's digest fallback calls request->authenticate(), which only exists under these
    # defines - web_server used to add them from its auth: block, which this design removes.
    # Digest specifically, because that is what the config shipped with ('type: digest') and what
    # every existing curl invocation and tuner script already speaks.
    cg.add_define("USE_WEBSERVER_AUTH")
    cg.add_define("USE_WEBSERVER_AUTH_DIGEST")

    if CONF_LOGIN_MIC_AVAILABLE in config:
        mic = await cg.process_lambda(
            config[CONF_LOGIN_MIC_AVAILABLE], [], return_type=cg.bool_
        )
        cg.add(var.set_login_mic_available(mic))

    if CONF_ON_LOGIN_WINDOW in config:
        await automation.build_automation(
            var.get_login_window_trigger(),
            [(cg.std_string, "mode"), (cg.std_string, "secret")],
            config[CONF_ON_LOGIN_WINDOW],
        )

    if CONF_ON_LOGIN_WINDOW_END in config:
        await automation.build_automation(
            var.get_login_window_end_trigger(),
            [(cg.std_string, "result")],
            config[CONF_ON_LOGIN_WINDOW_END],
        )

    # Behind a define, like the sendspin hub below: without it the handler never includes audio.h,
    # whose header only exists once some audio component is in the build.
    if config[CONF_SOUNDS]:
        cg.add_define("USE_SAT1_WEB_UI_SOUNDS", True)
        for name in sorted(config[CONF_SOUNDS]):
            sound = await cg.get_variable(config[CONF_SOUNDS][name])
            cg.add(var.add_sound(name, sound))

    # Sorted so the generated code, and the JSON the device serves, are stable across builds
    # regardless of how the YAML happens to be ordered.
    for group in sorted(config[CONF_ENTITIES]):
        domain, _ = _ENTITY_DOMAINS[group]
        for key in sorted(config[CONF_ENTITIES][group]):
            entity = await cg.get_variable(config[CONF_ENTITIES][group][key])
            cg.add(var.add_entity(key, domain, entity))

    if CONF_VOICE_ASSISTANT_ID in config:
        cg.add(var.set_voice_assistant(await cg.get_variable(config[CONF_VOICE_ASSISTANT_ID])))

    if CONF_VOICE_PHASE in config:
        phase = await cg.process_lambda(config[CONF_VOICE_PHASE], [], return_type=cg.int_)
        cg.add(var.set_voice_phase_fn(phase))

    if CONF_MICRO_WAKE_WORD_ID in config:
        cg.add(
            var.set_micro_wake_word(await cg.get_variable(config[CONF_MICRO_WAKE_WORD_ID]))
        )

    if CONF_WAKE_LOADER_ID in config:
        cg.add(var.set_wake_loader(await cg.get_variable(config[CONF_WAKE_LOADER_ID])))

    if CONF_CRASH_REPORT_ID in config:
        cg.add(var.set_crash_report(await cg.get_variable(config[CONF_CRASH_REPORT_ID])))

    if CONF_MEDIA_PLAYER_ID in config:
        cg.add(
            var.set_media_player(await cg.get_variable(config[CONF_MEDIA_PLAYER_ID]))
        )

    if CONF_SENDSPIN_MEDIA_PLAYER_ID in config:
        cg.add(
            var.set_sendspin_media_player(
                await cg.get_variable(config[CONF_SENDSPIN_MEDIA_PLAYER_ID])
            )
        )

    if CONF_SENDSPIN_HUB_ID in config:
        # A define rather than an unconditional include, so a config without sendspin never pulls
        # the hub header (whose own includes only exist once the sendspin IDF component is added).
        # The matching role requests happen in _request_sendspin_roles at validation time.
        cg.add_define("USE_SAT1_WEB_UI_SENDSPIN", True)
        cg.add(var.set_sendspin_hub(await cg.get_variable(config[CONF_SENDSPIN_HUB_ID])))

    if CONF_ON_SELECTION_CHANGE in config:
        await automation.build_automation(
            var.get_selection_change_trigger(), [], config[CONF_ON_SELECTION_CHANGE]
        )

    if CONF_ON_HA_REFRESH in config:
        await automation.build_automation(
            var.get_ha_refresh_trigger(), [], config[CONF_ON_HA_REFRESH]
        )

    if CONF_ON_HA_SELECT in config:
        await automation.build_automation(
            var.get_ha_select_trigger(),
            [(cg.std_string, "entity"), (cg.std_string, "option")],
            config[CONF_ON_HA_SELECT],
        )

    if CONF_ON_MA_REFRESH in config:
        await automation.build_automation(
            var.get_ma_refresh_trigger(), [], config[CONF_ON_MA_REFRESH]
        )

    if CONF_ON_MA_LIKE in config:
        await automation.build_automation(
            var.get_ma_like_trigger(), [(cg.std_string, "entity")], config[CONF_ON_MA_LIKE]
        )

    if CONF_ON_MA_JOIN in config:
        await automation.build_automation(
            var.get_ma_join_trigger(),
            [(cg.std_string, "entity"), (cg.std_string, "member")],
            config[CONF_ON_MA_JOIN],
        )

    if CONF_ON_MA_UNJOIN in config:
        await automation.build_automation(
            var.get_ma_unjoin_trigger(), [(cg.std_string, "entity")], config[CONF_ON_MA_UNJOIN]
        )

    if CONF_ON_MA_VOLUME in config:
        await automation.build_automation(
            var.get_ma_volume_trigger(),
            [(cg.std_string, "entity"), (cg.float_, "level")],
            config[CONF_ON_MA_VOLUME],
        )

    if CONF_ON_MA_SEEK in config:
        await automation.build_automation(
            var.get_ma_seek_trigger(),
            [(cg.std_string, "entity"), (cg.float_, "position")],
            config[CONF_ON_MA_SEEK],
        )

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

    if not _NO_SENSOR.is_file():
        raise cv.Invalid(f"The no-sensor image is missing at {_NO_SENSOR}.")

    # Not gzipped: WebP is already compressed, and a Content-Encoding layer would only add CPU at
    # request time for negative savings. PROGMEM like the bundle, so serving it costs no heap.
    webp = _NO_SENSOR.read_bytes()
    no_sensor = cg.progmem_array(config[CONF_NO_SENSOR_ID], tuple(map(HexInt, webp)))
    cg.add(var.set_no_sensor_image(no_sensor, len(webp)))

    # The PWA surface: home-screen icons committed as artifacts (like the photo above, so the CI
    # drift check compares reviewed bytes), and a manifest rendered here because it carries the
    # device's friendly name, which only codegen knows. Colors are the app's dark theme tokens
    # from frontend/src/app.css - the icons are drawn on the same background, so the home screen,
    # the splash and the app agree. iOS reads none of this (it wants the apple-touch-icon and the
    # meta tags in index.html); the manifest is for Chrome's add-to-home-screen path.
    manifest = json.dumps(
        {
            "name": CORE.friendly_name or CORE.name or "Satellite1",
            "short_name": "Satellite1",
            "start_url": "/",
            "display": "standalone",
            "background_color": "#111418",
            "theme_color": "#111418",
            "icons": [
                {"src": "/ui/icon-192.png", "sizes": "192x192", "type": "image/png"},
                {"src": "/ui/icon-512.png", "sizes": "512x512", "type": "image/png"},
            ],
        },
        separators=(",", ":"),
    ).encode()
    manifest_arr = cg.progmem_array(config[CONF_MANIFEST_ID], tuple(map(HexInt, manifest)))
    cg.add(var.set_manifest(manifest_arr, len(manifest)))

    for which, conf_key, name in (
        (0, CONF_ICON_192_ID, "icon_192.png"),
        (1, CONF_ICON_512_ID, "icon_512.png"),
        (2, CONF_ICON_180_ID, "icon_180.png"),
    ):
        icon_path = Path(__file__).parent / "assets" / name
        if not icon_path.is_file():
            raise cv.Invalid(f"The web app icon is missing at {icon_path}.")
        png = icon_path.read_bytes()
        arr = cg.progmem_array(config[conf_key], tuple(map(HexInt, png)))
        cg.add(var.set_icon(which, arr, len(png)))
