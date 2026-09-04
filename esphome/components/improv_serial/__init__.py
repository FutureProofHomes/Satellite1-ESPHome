import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import automation
from esphome.components import improv_base
from esphome.components.esp32 import VARIANT_ESP32S3, get_esp32_variant
from esphome.components.logger import USB_CDC
from esphome.const import CONF_BAUD_RATE, CONF_HARDWARE_UART, CONF_ID, CONF_LOGGER, CONF_WIFI
from esphome.core import CORE

AUTO_LOAD = ["improv_base", "network"]
DEPENDENCIES = ["logger"]

improv_serial_ns = cg.esphome_ns.namespace("improv_serial")

ImprovSerialComponent = improv_serial_ns.class_("ImprovSerialComponent", cg.Component)
ExtAction = improv_serial_ns.class_("ExtAction")
SendActionStatusAction = improv_serial_ns.class_("ImprovSendActionStatusAction", automation.Action)

CONF_ON_ACTION = "on_action"
CONF_ACTION_ID = "action"
CONF_STATUS = "status"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ImprovSerialComponent),
            cv.Optional(CONF_ON_ACTION): automation.validate_automation(single=True),
        }
    )
    .extend(improv_base.IMPROV_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


def validate_logger(config):
    logger_conf = fv.full_config.get()[CONF_LOGGER]
    if logger_conf[CONF_BAUD_RATE] == 0:
        raise cv.Invalid("improv_serial requires logger baud_rate to not be 0")
    if (
        CORE.is_esp32
        and logger_conf[CONF_HARDWARE_UART] == USB_CDC
        and get_esp32_variant() == VARIANT_ESP32S3
    ):
        raise cv.Invalid(
            "improv_serial does not support the selected logger hardware_uart"
        )
    if improv_base.CONF_NEXT_URL in config and CONF_WIFI not in fv.full_config.get():
        raise cv.Invalid("improv_serial next_url requires the wifi component")
    return config


FINAL_VALIDATE_SCHEMA = validate_logger


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await improv_base.setup_improv_core(var, config, "improv_serial")
    cg.add_define("USE_IMPROV_SERIAL")

    if on_action := config.get(CONF_ON_ACTION):
        await automation.build_automation(
            var.get_action_request_trigger(), [(ExtAction, "x")], on_action
        )


SEND_ACTION_STATUS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.use_id(ImprovSerialComponent),
        cv.Required(CONF_ACTION_ID): cv.string,
        cv.Required(CONF_STATUS): cv.positive_int,
    }
)


@automation.register_action(
    "improv_serial.send_action_status",
    SendActionStatusAction,
    SEND_ACTION_STATUS_SCHEMA,
    synchronous=True,
)
async def send_action_status_to_code(config, action_id, template_args, args):
    improv_comp = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_args, improv_comp)
    action = await cg.templatable(config[CONF_ACTION_ID], args, cg.std_string)
    cg.add(var.set_action(action))
    status = await cg.templatable(config[CONF_STATUS], args, cg.int_)
    cg.add(var.set_status(status))
    return var
