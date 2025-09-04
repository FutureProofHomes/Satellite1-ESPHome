import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.core import CORE


DEPENDENCIES = ["network", "audio"]
CODEOWNERS = ["@gnumpi"]

CONF_SERVER_IP = "server_ip"

def AUTO_LOAD():
    if CORE.is_esp8266 or CORE.is_libretiny:
        return ["async_tcp", "json", "socket"]
    return ["json", "socket"]

snapcast_ns = cg.esphome_ns.namespace("snapcast")
SnapcastClient = snapcast_ns.class_("SnapcastClient", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SnapcastClient),
    cv.Optional(CONF_SERVER_IP) : cv.ipaddress
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    if CONF_SERVER_IP in config :
        cg.add(var.set_server_ip(str(config[CONF_SERVER_IP])))    
    
    cg.add_define("USE_SNAPCAST", True)
    cg.add_define("SNAPCAST_DEBUG", False)