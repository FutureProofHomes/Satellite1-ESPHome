import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation
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


EnableAction = snapcast_ns.class_(
    "EnableAction", automation.Action, cg.Parented.template(SnapcastClient)
)

DisableAction = snapcast_ns.class_(
    "DisableAction", automation.Action, cg.Parented.template(SnapcastClient)
)

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


SNAPCAST_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(SnapcastClient),
    }
)


@automation.register_action("snapcast.enable", EnableAction, SNAPCAST_ACTION_SCHEMA)
@automation.register_action("snapcast.disable", DisableAction, SNAPCAST_ACTION_SCHEMA)
async def tas2780_action(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var