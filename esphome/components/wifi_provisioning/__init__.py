"""Reject YAML WiFi credentials when Satellite1 Improv provisioning is enabled."""

import esphome.config_validation as cv
from esphome.const import CONF_NETWORKS, CONF_WIFI
import esphome.final_validate as fv

DEPENDENCIES = ["wifi", "esp32_improv"]

CONFIG_SCHEMA = cv.All(cv.Schema({}), cv.only_on_esp32)


def _validate_no_networks(config):
    if fv.full_config.get()[CONF_WIFI].get(CONF_NETWORKS):
        raise cv.Invalid(
            "WiFi provisioning cannot be combined with YAML WiFi credentials. "
            "Remove the provisioning package to use wifi.ssid or wifi.networks."
        )
    return config


FINAL_VALIDATE_SCHEMA = _validate_no_networks


async def to_code(config):
    pass
