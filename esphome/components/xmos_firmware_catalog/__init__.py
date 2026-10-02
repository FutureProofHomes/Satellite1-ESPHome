import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import button, esp32, select, text_sensor
from esphome.components.satellite1.memory_flasher import XMOSFlasher
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["network", "http_request", "memory_flasher", "satellite1"]
AUTO_LOAD = ["button", "json", "md5", "select", "text_sensor"]

CONF_FLASHER_ID = "flasher_id"
CONF_BUILTIN_VERSION = "builtin_version"
CONF_SOURCES = "sources"
CONF_GITHUB = "github"
CONF_INCLUDE_PRERELEASES = "include_prereleases"
CONF_PRIVATE = "private"
CONF_TOKEN = "token"
CONF_MAX_RELEASES = "max_releases"
CONF_FIRMWARE = "firmware"
CONF_REFRESH = "refresh"
CONF_INSTALL = "install"
CONF_CATALOG = "catalog"
CONF_STATUS = "status"
CONF_ON_INSTALL_REQUEST = "on_install_request"
CONF_ON_CHOICES_CHANGED = "on_choices_changed"

DEFAULT_SOURCE = "FutureProofHomes/Satellite1-XMOS"

xmos_firmware_catalog_ns = cg.esphome_ns.namespace("xmos_firmware_catalog")
XmosFirmwareCatalog = xmos_firmware_catalog_ns.class_(
    "XmosFirmwareCatalog", cg.Component
)
CatalogSelect = xmos_firmware_catalog_ns.class_(
    "CatalogSelect", select.Select, cg.Parented.template(XmosFirmwareCatalog)
)
CatalogRefreshButton = xmos_firmware_catalog_ns.class_(
    "CatalogRefreshButton", button.Button, cg.Parented.template(XmosFirmwareCatalog)
)
CatalogInstallButton = xmos_firmware_catalog_ns.class_(
    "CatalogInstallButton", button.Button, cg.Parented.template(XmosFirmwareCatalog)
)
InstallRequestTrigger = xmos_firmware_catalog_ns.class_(
    "InstallRequestTrigger", automation.Trigger.template(cg.std_string)
)
ChoicesChangedTrigger = xmos_firmware_catalog_ns.class_(
    "ChoicesChangedTrigger", automation.Trigger.template()
)

# Matches the version format the XMOS reports and memory_flasher embeds.
VERSION_RE = re.compile(
    r"^v?(\d+)\.(\d+)\.(\d+)(-(alpha|beta|rc|dev)(\.(\d+))?)?$", re.IGNORECASE
)


def validate_version(value):
    value = cv.string_strict(value)
    match = VERSION_RE.match(value)
    if match is None:
        raise cv.Invalid(f"'{value}' is not an XMOS version like v1.2.3 or v1.2.3-dev.4")
    numbers = [match.group(1), match.group(2), match.group(3), match.group(7) or "0"]
    if any(int(number) > 255 for number in numbers):
        raise cv.Invalid("XMOS version numbers must be 255 or lower")
    return value


def validate_repo(value):
    value = cv.string_strict(value)
    if not re.fullmatch(r"[A-Za-z0-9-]+/[A-Za-z0-9._-]+", value):
        raise cv.Invalid("Expected a GitHub repository like 'owner/repo'")
    return value


SOURCE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_GITHUB): validate_repo,
        cv.Optional(CONF_INCLUDE_PRERELEASES, default=True): cv.boolean,
        cv.Optional(CONF_PRIVATE, default=False): cv.boolean,
        cv.Optional(CONF_TOKEN): cv.All(cv.string_strict, cv.Length(min=1, max=159)),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(XmosFirmwareCatalog),
            cv.Required(CONF_FLASHER_ID): cv.use_id(XMOSFlasher),
            cv.Required(CONF_BUILTIN_VERSION): validate_version,
            cv.Optional(
                CONF_SOURCES, default=[{CONF_GITHUB: DEFAULT_SOURCE}]
            ): cv.All(cv.ensure_list(SOURCE_SCHEMA), cv.Length(min=1, max=4)),
            cv.Optional(CONF_MAX_RELEASES, default=20): cv.int_range(min=1, max=100),
            cv.Optional(CONF_FIRMWARE): select.select_schema(
                CatalogSelect,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:chip",
            ),
            cv.Optional(CONF_REFRESH): button.button_schema(
                CatalogRefreshButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:refresh",
            ),
            cv.Optional(CONF_INSTALL): button.button_schema(
                CatalogInstallButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:download",
            ),
            cv.Optional(CONF_CATALOG): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:format-list-bulleted",
            ),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:progress-download",
            ),
            cv.Optional(CONF_ON_INSTALL_REQUEST): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        InstallRequestTrigger
                    ),
                }
            ),
            # Without this automation the API connection is dropped instead, so Home
            # Assistant reconnects and reads the new choices.
            cv.Optional(CONF_ON_CHOICES_CHANGED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        ChoicesChangedTrigger
                    ),
                }
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    flasher = await cg.get_variable(config[CONF_FLASHER_ID])
    cg.add(var.set_flasher(flasher))
    cg.add(var.set_builtin_version(config[CONF_BUILTIN_VERSION]))
    cg.add(var.set_max_releases(config[CONF_MAX_RELEASES]))
    for source in config[CONF_SOURCES]:
        cg.add(
            var.add_source(
                source[CONF_GITHUB],
                source[CONF_INCLUDE_PRERELEASES],
                source[CONF_PRIVATE],
                source.get(CONF_TOKEN, ""),
            )
        )

    if select_config := config.get(CONF_FIRMWARE):
        sel = await select.new_select(
            select_config, options=[f"Built-in ({config[CONF_BUILTIN_VERSION]})"]
        )
        await cg.register_parented(sel, var)
        cg.add(var.set_select(sel))

    if refresh_config := config.get(CONF_REFRESH):
        btn = await button.new_button(refresh_config)
        await cg.register_parented(btn, var)

    if install_config := config.get(CONF_INSTALL):
        btn = await button.new_button(install_config)
        await cg.register_parented(btn, var)

    if catalog_config := config.get(CONF_CATALOG):
        sens = await text_sensor.new_text_sensor(catalog_config)
        cg.add(var.set_catalog_text_sensor(sens))

    if status_config := config.get(CONF_STATUS):
        sens = await text_sensor.new_text_sensor(status_config)
        cg.add(var.set_status_text_sensor(sens))

    for conf in config.get(CONF_ON_INSTALL_REQUEST, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "xmos_version")], conf)

    for conf in config.get(CONF_ON_CHOICES_CHANGED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_define("USE_XMOS_FIRMWARE_CATALOG")
    esp32.include_builtin_idf_component("esp_http_client")
    esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_CERTIFICATE_BUNDLE", True)
