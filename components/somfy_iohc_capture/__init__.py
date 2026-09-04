import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

from esphome.components.somfy import SomfyIohcHub

DEPENDENCIES = ["somfy"]
AUTO_LOAD = ["text_sensor"]

capture_ns = cg.esphome_ns.namespace("somfy")
SomfyIohcCapture = capture_ns.class_("SomfyIohcCapture", cg.Component)

CONF_SOMFY_ID = "somfy_id"
CONF_CAPTURE = "capture"
CONF_REMOTE_CODE = "remote_code"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SomfyIohcCapture),
        cv.Required(CONF_SOMFY_ID): cv.use_id(SomfyIohcHub),
        cv.Required(CONF_CAPTURE): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_REMOTE_CODE): cv.hex_int_range(min=1, max=0xFFFFFF),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_SOMFY_ID])
    sensor = await text_sensor.new_text_sensor(config[CONF_CAPTURE])
    cg.add(var.set_hub(hub))
    cg.add(var.set_capture_sensor(sensor))

    if CONF_REMOTE_CODE in config:
        cg.add(var.set_remote_code(config[CONF_REMOTE_CODE]))

    cg.add_define("USE_SOMFY_IOHC_CAPTURE")
