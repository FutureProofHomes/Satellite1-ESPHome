import logging
import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

from esphome.components.memory_flasher import (
    CONF_EMBED_FLASH_IMAGE,
    CONF_FLASH_IMAGE_VERSION,
    FLASHER_CONFIG_SCHEMA,
    MemoryFlasher,
    register_memory_flasher
)

from .. import satellite1 as sat

CODEOWNERS = ["@gnumpi"]

DEPENDENCIES = ["satellite1", "memory_flasher"]

MINIMUM_XMOS_FIRMWARE_VERSION = "v1.1.0-alpha.0"
CONF_SKIP_VERSION_CHECK = "skip_version_check"

_LOGGER = logging.getLogger(__name__)
_VERSION_PATTERN = re.compile(
    r"v?(\d+)\.(\d+)\.(\d+)(?:-(alpha|beta|rc|dev)(?:\.(\d+))?)?",
    re.IGNORECASE,
)
_PRERELEASE_ORDER = {"dev": 0, "alpha": 1, "beta": 2, "rc": 3}


def _version_key(value):
    value = cv.string_strict(value)
    match = _VERSION_PATTERN.fullmatch(value)
    if match is None:
        raise cv.Invalid(f"Not a valid version string: {value}")
    major, minor, patch, prerelease, iteration = match.groups()
    # Stable releases sort after all prereleases of the same numeric version.
    return (
        int(major), int(minor), int(patch),
        _PRERELEASE_ORDER[prerelease.lower()] if prerelease else 4,
        int(iteration or 0),
    )


def _validate_minimum_version(config):
    # Run on resolved configuration before the generic schema downloads images
    # or checksum files. The generic schema still validates every other field.
    if not isinstance(config, dict):
        return config
    skip = cv.boolean(config.get(CONF_SKIP_VERSION_CHECK, False))
    image = config.get(CONF_EMBED_FLASH_IMAGE)
    if isinstance(image, dict) and CONF_FLASH_IMAGE_VERSION in image:
        version = image[CONF_FLASH_IMAGE_VERSION]
        try:
            version_key = _version_key(version)
            if not skip and version_key < _version_key(MINIMUM_XMOS_FIRMWARE_VERSION):
                raise cv.Invalid(
                    f"XMOS firmware {version} is below the Satellite1 minimum "
                    f"{MINIMUM_XMOS_FIRMWARE_VERSION}. Update "
                    "substitutions.xmos_fw_version to "
                    f"{MINIMUM_XMOS_FIRMWARE_VERSION} or newer, or remove the stale override."
                )
        except cv.Invalid as err:
            raise cv.Invalid(
                str(err), path=[CONF_EMBED_FLASH_IMAGE, CONF_FLASH_IMAGE_VERSION]
            ) from err
    if skip:
        _LOGGER.warning(
            "Satellite1 XMOS compatibility-bypass: skip_version_check=true disables "
            "the minimum firmware version check; compatibility is not guaranteed. "
            "Version syntax, checksum validation and flashing safeguards remain enabled."
        )
    return config

XMOSFlasher = sat.namespace.class_("XMOSFlasher", MemoryFlasher, sat.Satellite1SPIService, cg.Component )



CONFIG_SCHEMA = cv.All(
    _validate_minimum_version,
    FLASHER_CONFIG_SCHEMA.extend(
        cv.Schema(
            {
                cv.GenerateID(): cv.declare_id(XMOSFlasher),
                cv.GenerateID(sat.CONF_SATELLITE1): cv.use_id(sat.Satellite1),
                cv.Optional(CONF_SKIP_VERSION_CHECK, default=False): cv.boolean,
            }
        )
    ),
)
    


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await register_memory_flasher(var, config)
    await cg.register_parented(var, config[sat.CONF_SATELLITE1])
    return var
