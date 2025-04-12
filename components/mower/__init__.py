import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL
from esphome.components import uart

# Create the namespace and component class.
mower_ns = cg.esphome_ns.namespace("mower")
Automower = mower_ns.class_("Automower", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Automower),
    cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    uart_var = config["uart_id"]
    update_interval = config[CONF_UPDATE_INTERVAL]
    var = cg.new_Pvariable(config[CONF_ID], uart_var, update_interval)
    cg.register_component(var, config)
