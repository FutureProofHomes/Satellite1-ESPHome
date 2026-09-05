from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import network
from esphome.components.esp32 import add_idf_component
from esphome.const import CONF_DURATION, CONF_ID

CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["network"]

iperf_ns = cg.esphome_ns.namespace("iperf")
Iperf = iperf_ns.class_("Iperf", cg.Component)

StartClientAction = iperf_ns.class_(
    "IPerfStartClientAction", automation.Action, cg.Parented.template(Iperf)
)

CONF_SERVER = "server"
CONF_HIGH_PERFORMANCE_NET = "high_performance_net"


def _request_high_performance_networking(config):
    if config[CONF_HIGH_PERFORMANCE_NET]:
        network.require_high_performance_networking()
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Iperf),
            cv.Optional(CONF_HIGH_PERFORMANCE_NET, default=False): cv.boolean,
        }
    ),
    _request_high_performance_networking,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    add_idf_component(name="espressif/iperf", ref="0.1.4")


IPERF_CLIENT_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(Iperf),
        cv.Required(CONF_SERVER): cv.templatable(cv.string_strict),
        cv.Optional(CONF_DURATION, default=0): cv.templatable(cv.uint32_t),
    }
)


@automation.register_action(
    "iperf.start_client", StartClientAction, IPERF_CLIENT_ACTION_SCHEMA, synchronous=True
)
async def iperf_register_client_actions(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    server = await cg.templatable(config[CONF_SERVER], args, cg.std_string)
    cg.add(var.set_server(server))
    duration = await cg.templatable(config[CONF_DURATION], args, cg.uint32)
    cg.add(var.set_duration(duration))
    return var
