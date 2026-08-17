import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import remote_receiver, remote_transmitter
from esphome.const import CONF_ID, CONF_TYPE, PLATFORM_ESP32

CODEOWNERS = ["@LeonardPitzu"]
DEPENDENCIES = ["esp32"]
# RX state synchronisation publishes a detected-remote sensor when configured,
# and the same RX implementation is also used to model a native MY recall. Load
# the lightweight text_sensor base unconditionally so every valid combination
# of those optional fields has its C++ headers available.
AUTO_LOAD = ["button", "text_sensor"]
MULTI_CONF = True

DOMAIN = "somfy"

somfy_ns = cg.esphome_ns.namespace("somfy")
SomfyRtsHub = somfy_ns.class_("SomfyRtsHub", cg.Component)
SomfyIohcHub = somfy_ns.class_("SomfyIohcHub", cg.Component)

CONF_REMOTE_TRANSMITTER = "remote_transmitter"
CONF_REMOTE_RECEIVER = "remote_receiver"
CONF_CC1101_ID = "cc1101_id"

TYPE_RTS = "rts"
TYPE_IOHC = "iohc"

RTS_HUB_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SomfyRtsHub),
        cv.Required(CONF_REMOTE_TRANSMITTER): cv.use_id(
            remote_transmitter.RemoteTransmitterComponent
        ),
        cv.Optional(CONF_REMOTE_RECEIVER): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

IOHC_HUB_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SomfyIohcHub),
        cv.Required(CONF_CC1101_ID): cv.use_id(cg.Component),
    }
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            TYPE_RTS: RTS_HUB_SCHEMA,
            TYPE_IOHC: IOHC_HUB_SCHEMA,
        },
    ),
    cv.only_on([PLATFORM_ESP32]),
)


async def to_code(config):
    typ = config[CONF_TYPE]

    if typ == TYPE_RTS:
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)
        cg.add_define("USE_SOMFY_RTS")

        tx = await cg.get_variable(config[CONF_REMOTE_TRANSMITTER])
        cg.add(var.set_remote_transmitter(tx))

        if CONF_REMOTE_RECEIVER in config:
            rx = await cg.get_variable(config[CONF_REMOTE_RECEIVER])
            cg.add(var.set_remote_receiver(rx))
            cg.add_define("USE_SOMFY_COVER_RX")

    elif typ == TYPE_IOHC:
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)

        cc1101 = await cg.get_variable(config[CONF_CC1101_ID])
        cg.add(var.set_cc1101(cc1101))
        cg.add_define("USE_SOMFY_IOHC")
