import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_CLOSE_DURATION,
    CONF_ID,
    CONF_OPEN_DURATION,
)
from esphome.core import CORE
from esphome.core.entity_helpers import register_device_class

from esphome.components.somfy import SomfyIohcHub

DEPENDENCIES = ["api", "esp32", "somfy"]
AUTO_LOAD = ["cover", "text_sensor"]

manager_ns = cg.esphome_ns.namespace("somfy")
SomfyIohcManager = manager_ns.class_("SomfyIohcManager", cg.Component)

CONF_SOMFY_ID = "somfy_id"
CONF_STATUS_SENSOR = "status_sensor"
CONF_BACKUP_SENSOR = "backup_sensor"
CONF_BACKUP_KEY = "backup_key"
CONF_RELAY_KEY = "relay_key"
CONF_MAX_SHUTTERS = "max_shutters"
CONF_IMPORTS = "imports"
CONF_SLOT = "slot"
CONF_REMOTE_CODE = "remote_code"
CONF_ENCRYPTION_KEY = "encryption_key"
CONF_STORAGE_NAMESPACE = "storage_namespace"
CONF_STORAGE_KEY = "storage_key"
CONF_INITIAL_ROLLING_CODE = "initial_rolling_code"
CONF_PHYSICAL_REMOTE = "physical_remote"
CONF_MY_POSITION = "my_position"
CONF_REPEAT_COMMAND_COUNT = "repeat_command_count"
DEFAULT_MAX_SHUTTERS = 20


def validate_hex_key(value):
    value = cv.string(value)
    if len(value) != 32 or any(ch not in "0123456789abcdefABCDEF" for ch in value):
        raise cv.Invalid("key must contain exactly 32 hexadecimal characters")
    return value


def mark_sensitive(validator):
    """Use deterministic ESPHome 2026.7+ redaction without dropping 2026.4 support."""
    sensitive = getattr(cv, "sensitive", None)
    return sensitive(validator) if callable(sensitive) else validator


IMPORT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SLOT): cv.int_range(min=0, max=31),
        cv.Required(CONF_REMOTE_CODE): cv.hex_int_range(min=1, max=0xFFFFFF),
        cv.Required(CONF_ENCRYPTION_KEY): mark_sensitive(validate_hex_key),
        cv.Required(CONF_STORAGE_KEY): mark_sensitive(
            cv.All(cv.string, cv.Length(max=15))
        ),
        cv.Optional(CONF_STORAGE_NAMESPACE, default="somfy"): cv.All(
            cv.string, cv.Length(max=15)
        ),
        cv.Optional(CONF_INITIAL_ROLLING_CODE, default=1): cv.hex_int_range(
            min=1, max=0xFFFF
        ),
        cv.Required(CONF_PHYSICAL_REMOTE): cv.hex_int_range(min=1, max=0xFFFFFF),
        cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
        cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_MY_POSITION, default="50%"): cv.percentage,
        cv.Optional(CONF_REPEAT_COMMAND_COUNT, default=6): cv.int_range(min=1, max=100),
    }
)


def validate_manager(config):
    slots = [item[CONF_SLOT] for item in config.get(CONF_IMPORTS, [])]
    if len(slots) != len(set(slots)):
        raise cv.Invalid("imported shutter slots must be unique")
    max_shutters = config[CONF_MAX_SHUTTERS]
    if any(slot >= max_shutters for slot in slots):
        raise cv.Invalid("an imported shutter slot is outside max_shutters")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SomfyIohcManager),
            cv.Required(CONF_SOMFY_ID): cv.use_id(SomfyIohcHub),
            cv.Required(CONF_STATUS_SENSOR): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_BACKUP_SENSOR): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_BACKUP_KEY): mark_sensitive(validate_hex_key),
            # Cluster transport key used only to authenticate/encrypt opaque
            # exact-frame relay envelopes. It never replaces a shutter's
            # controller key. Defaulting to backup_key keeps existing manager
            # YAML valid while allowing operators to rotate it independently.
            cv.Optional(CONF_RELAY_KEY): mark_sensitive(validate_hex_key),
            cv.Optional(CONF_MAX_SHUTTERS, default=DEFAULT_MAX_SHUTTERS): cv.int_range(
                min=1, max=32
            ),
            cv.Optional(CONF_IMPORTS, default=[]): cv.ensure_list(IMPORT_SCHEMA),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_manager,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_SOMFY_ID])
    cg.add(var.set_hub(hub))

    status = await cg.get_variable(config[CONF_STATUS_SENSOR])
    backup = await cg.get_variable(config[CONF_BACKUP_SENSOR])
    cg.add(var.set_status_sensor(status))
    cg.add(var.set_backup_sensor(backup))
    cg.add(var.set_backup_key(config[CONF_BACKUP_KEY]))
    cg.add(var.set_relay_key(config.get(CONF_RELAY_KEY, config[CONF_BACKUP_KEY])))
    cg.add(var.set_max_shutters(config[CONF_MAX_SHUTTERS]))
    cg.add(var.set_cover_device_class_index(register_device_class("shutter")))

    for item in config[CONF_IMPORTS]:
        cg.add(
            var.add_import(
                item[CONF_SLOT],
                item[CONF_REMOTE_CODE],
                item[CONF_ENCRYPTION_KEY],
                item[CONF_STORAGE_NAMESPACE],
                item[CONF_STORAGE_KEY],
                item[CONF_INITIAL_ROLLING_CODE],
                item[CONF_PHYSICAL_REMOTE],
                item[CONF_OPEN_DURATION],
                item[CONF_CLOSE_DURATION],
                item[CONF_MY_POSITION],
                item[CONF_REPEAT_COMMAND_COUNT],
            )
        )

    # Slots are registered during generated setup(), before App.setup() starts.
    for _ in range(config[CONF_MAX_SHUTTERS]):
        CORE.register_platform_component("cover", var)
    cg.add(var.create_slots())
    cg.add_define("USE_SOMFY_IOHC_MANAGER")
    cg.add_define("USE_SOMFY_IOHC_RX")
