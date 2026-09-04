import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import button, cover, text_sensor
from esphome.const import (
    CONF_CLOSE_DURATION,
    CONF_ID,
    CONF_OPEN_DURATION,
    CONF_TYPE,
    PLATFORM_ESP32,
)

from . import (
    CONF_REMOTE_RECEIVER,
    DOMAIN,
    SomfyIohcHub,
    SomfyRtsHub,
    somfy_ns,
)

CODEOWNERS = ["@LeonardPitzu"]
DEPENDENCIES = ["esp32"]

SomfyCover = somfy_ns.class_("SomfyCover", cover.Cover, cg.Component)
SomfyIohcCover = somfy_ns.class_("SomfyIohcCover", cover.Cover, cg.Component)
# IohcMode is a C++ `enum class`, so codegen must scope it (IohcMode::MODE_1W).
IohcMode = somfy_ns.enum("IohcMode", is_class=True)

# Shared config keys
CONF_SOMFY_ID = "somfy_id"
CONF_REMOTE_CODE = "remote_code"
CONF_SOMFY_STORAGE_KEY = "storage_key"
CONF_SOMFY_STORAGE_NAMESPACE = "storage_namespace"
CONF_REPEAT_COMMAND_COUNT = "repeat_command_count"
CONF_PROG_BUTTON = "prog_button"
CONF_INITIAL_ROLLING_CODE = "initial_rolling_code"

# RTS-specific
CONF_ALLOWED_REMOTES = "allowed_remotes"
CONF_DETECTED_REMOTE = "detected_remote"

# iohc-specific
CONF_ENCRYPTION_KEY = "encryption_key"
CONF_IOHC_MODE = "mode"
CONF_TARGET_NODE = "target_node"
CONF_MY_BUTTON = "my_button"
CONF_MY_POSITION = "my_position"

TYPE_RTS = "rts"
TYPE_IOHC = "iohc"

# iohc mode constants
IOHC_MODE_1W = "1w"
IOHC_MODE_2W = "2w"


def validate_iohc_config(config):
    """Validate iohc-specific config: target_node required for 2W mode."""
    mode = config.get(CONF_IOHC_MODE, IOHC_MODE_1W)
    if mode == IOHC_MODE_2W:
        if CONF_TARGET_NODE not in config:
            raise cv.Invalid(
                f"'{CONF_TARGET_NODE}' is required when mode is '{IOHC_MODE_2W}'"
            )
        if CONF_ENCRYPTION_KEY not in config:
            raise cv.Invalid(
                f"'{CONF_ENCRYPTION_KEY}' (system key) is required for 2W mode"
            )
    if CONF_MY_BUTTON in config and CONF_MY_POSITION not in config:
        raise cv.Invalid(
            f"'{CONF_MY_POSITION}' is required when '{CONF_MY_BUTTON}' is configured"
        )
    return config


def validate_encryption_key(value):
    """Require exactly 16 bytes encoded as 32 hexadecimal characters."""
    value = cv.string(value)
    if len(value) != 32 or any(ch not in "0123456789abcdefABCDEF" for ch in value):
        raise cv.Invalid("encryption_key must contain exactly 32 hexadecimal characters")
    return value


def uses_rx(config):
    """Return True if any RX-related field is configured on a cover.

    Note: remote_receiver is set on the hub, not the cover.
    This helper checks the cover-level fields (allowed_remotes, detected_remote).
    """
    if config.get(CONF_DETECTED_REMOTE):
        return True
    return bool(config.get(CONF_ALLOWED_REMOTES))


def iohc_uses_rx(config):
    """Return True if an iohc cover needs the RX state-sync code compiled in.

    io-homecontrol 2W is bidirectional by definition -- the node reports back --
    so RX is always built in for it. 1W is a broadcast-only protocol like RTS,
    so there RX is opt-in via allowed_remotes/detected_remote.
    """
    if config.get(CONF_IOHC_MODE, IOHC_MODE_1W) == IOHC_MODE_2W:
        return True
    return uses_rx(config) or CONF_MY_POSITION in config


def validate_rts_config(config, hub_config=None):
    """Validate an RTS cover against the hub it references.

    ``allowed_remotes`` / ``detected_remote`` only do anything when the hub owns
    a ``remote_receiver``; without one the RX code is not compiled in at all.
    ``hub_config`` is the referenced hub's config, or None when it cannot be
    resolved (in which case validation is skipped rather than guessed at).
    """
    if not uses_rx(config):
        return config
    # The optional second argument is used by final validation, where the hub
    # is available separately. Keeping the one-argument form preserves the
    # public validator API and supports direct/unit validation of legacy maps
    # that include remote_receiver inline.
    effective_hub = config if hub_config is None else hub_config
    if CONF_REMOTE_RECEIVER in effective_hub:
        return config
    raise cv.Invalid(
        f"'{CONF_ALLOWED_REMOTES}' and '{CONF_DETECTED_REMOTE}' need the somfy "
        f"hub to have a '{CONF_REMOTE_RECEIVER}' configured"
    )


def find_hub_config(full_config, hub_id):
    """Return the somfy hub config with the given id, or None if absent."""
    hubs = full_config.get(DOMAIN) or []
    if isinstance(hubs, dict):  # not MULTI_CONF (shouldn't happen, be lenient)
        hubs = [hubs]
    for hub in hubs:
        if str(hub.get(CONF_ID)) == str(hub_id):
            return hub
    return None


# Common fields shared by both RTS and iohc covers
COMMON_COVER_FIELDS = {
    cv.Required(CONF_PROG_BUTTON): cv.use_id(button.Button),
    cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
    cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
    cv.Required(CONF_REMOTE_CODE): cv.hex_int_range(min=1, max=0xFFFFFF),
    cv.Required(CONF_SOMFY_STORAGE_KEY): cv.All(cv.string, cv.Length(max=15)),
    cv.Optional(CONF_SOMFY_STORAGE_NAMESPACE, default="somfy"): cv.All(
        cv.string, cv.Length(max=15)
    ),
    cv.Optional(CONF_REPEAT_COMMAND_COUNT, default=4): cv.int_range(min=1, max=100),
    # Used only when the NVS key does not exist (fresh/replacement ESP). Zero is
    # reserved as a storage-error sentinel in C++.
    cv.Optional(CONF_INITIAL_ROLLING_CODE, default=1): cv.hex_int_range(
        min=1, max=0xFFFF
    ),
}

RTS_COVER_SCHEMA = (
    cover.cover_schema(SomfyCover)
    .extend(
        {
            cv.Required(CONF_SOMFY_ID): cv.use_id(SomfyRtsHub),
            cv.Optional(CONF_ALLOWED_REMOTES, default=[]): cv.ensure_list(
                cv.hex_uint32_t
            ),
            cv.Optional(CONF_DETECTED_REMOTE): cv.use_id(text_sensor.TextSensor),
        }
    )
    .extend(COMMON_COVER_FIELDS)
    .extend(cv.COMPONENT_SCHEMA)
)

IOHC_COVER_SCHEMA = cv.All(
    cover.cover_schema(SomfyIohcCover)
    .extend(
        {
            cv.Required(CONF_SOMFY_ID): cv.use_id(SomfyIohcHub),
            cv.Optional(CONF_ENCRYPTION_KEY): validate_encryption_key,
            cv.Optional(CONF_IOHC_MODE, default=IOHC_MODE_1W): cv.one_of(
                IOHC_MODE_1W, IOHC_MODE_2W, lower=True
            ),
            cv.Optional(CONF_TARGET_NODE): cv.hex_uint32_t,
            cv.Optional(CONF_MY_POSITION): cv.percentage,
            cv.Optional(CONF_MY_BUTTON): cv.use_id(button.Button),
            # RX state-sync: learn physical io-homecontrol remote IDs and keep
            # HA in sync when a motor is driven by an original remote. The iohc
            # hub always listens (CC1101 sits in RX), so unlike RTS no separate
            # receiver is required.
            cv.Optional(CONF_ALLOWED_REMOTES, default=[]): cv.ensure_list(
                cv.hex_uint32_t
            ),
            cv.Optional(CONF_DETECTED_REMOTE): cv.use_id(text_sensor.TextSensor),
        }
    )
    .extend(COMMON_COVER_FIELDS)
    .extend(cv.COMPONENT_SCHEMA),
    validate_iohc_config,
)

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            TYPE_RTS: RTS_COVER_SCHEMA,
            TYPE_IOHC: IOHC_COVER_SCHEMA,
        },
    ),
    cv.only_on([PLATFORM_ESP32]),
)


def _final_validate(config):
    # remote_receiver lives on the hub, so this cross-component check can only
    # run once the whole config is known. Without it a cover configured with
    # allowed_remotes against a receiver-less hub fails later with a cryptic C++
    # compile error instead of a config message.
    if config[CONF_TYPE] == TYPE_RTS:
        hub_config = find_hub_config(fv.full_config.get(), config[CONF_SOMFY_ID])
        validate_rts_config(config, hub_config)

    # A rolling-code key identifies one monotonically increasing stream. Two
    # independently configured cover entities must never accidentally reuse it.
    # Likewise, duplicate IOHC controller IDs would make broadcast commands a
    # group operation; represent an intentional group as one cover entity.
    all_covers = fv.full_config.get().get("cover") or []
    this_id = str(config.get(CONF_ID))
    this_storage = (
        config.get(CONF_SOMFY_STORAGE_NAMESPACE, "somfy"),
        config.get(CONF_SOMFY_STORAGE_KEY),
    )
    for other in all_covers:
        if str(other.get(CONF_ID)) == this_id or CONF_SOMFY_ID not in other:
            continue
        other_storage = (
            other.get(CONF_SOMFY_STORAGE_NAMESPACE, "somfy"),
            other.get(CONF_SOMFY_STORAGE_KEY),
        )
        if this_storage == other_storage:
            raise cv.Invalid(
                "Somfy covers must not share storage_namespace/storage_key; "
                f"duplicate {this_storage[0]}/{this_storage[1]}"
            )
        if (
            config[CONF_TYPE] == TYPE_IOHC
            and other.get(CONF_TYPE) == TYPE_IOHC
            and config.get(CONF_REMOTE_CODE) == other.get(CONF_REMOTE_CODE)
        ):
            raise cv.Invalid(
                "IOHC covers must not reuse remote_code; use one cover entity "
                "for an intentional broadcast group"
            )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    # SomfyCover / SomfyIohcCover extend the component-local SomfyTimeBasedCover
    # base (vendored from ESPHome's time_based cover, which became `final` in
    # 2026.07). The C++ sources ship inside this component, so no external
    # sub-platform needs to be pulled into the build tree.
    typ = config[CONF_TYPE]

    if typ == TYPE_RTS:
        await _to_code_rts(config)
    elif typ == TYPE_IOHC:
        await _to_code_iohc(config)


async def _to_code_rts(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    hub = await cg.get_variable(config[CONF_SOMFY_ID])
    cg.add(var.set_hub(hub))

    btn = await cg.get_variable(config[CONF_PROG_BUTTON])
    cg.add(var.set_prog_button(btn))

    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_remote_code(config[CONF_REMOTE_CODE]))
    cg.add(var.set_storage_key(config[CONF_SOMFY_STORAGE_KEY]))
    cg.add(var.set_storage_namespace(config[CONF_SOMFY_STORAGE_NAMESPACE]))
    cg.add(var.set_initial_rolling_code(config[CONF_INITIAL_ROLLING_CODE]))
    cg.add(var.set_repeat_count(config[CONF_REPEAT_COMMAND_COUNT]))

    if CONF_ALLOWED_REMOTES in config:
        for code in config[CONF_ALLOWED_REMOTES]:
            cg.add(var.add_receive_remote_code(code))

    if CONF_DETECTED_REMOTE in config:
        log_sensor = await cg.get_variable(config[CONF_DETECTED_REMOTE])
        cg.add(var.set_log_text_sensor(log_sensor))


async def _to_code_iohc(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    hub = await cg.get_variable(config[CONF_SOMFY_ID])
    cg.add(var.set_hub(hub))

    btn = await cg.get_variable(config[CONF_PROG_BUTTON])
    cg.add(var.set_prog_button(btn))

    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_remote_code(config[CONF_REMOTE_CODE]))
    cg.add(var.set_storage_key(config[CONF_SOMFY_STORAGE_KEY]))
    cg.add(var.set_storage_namespace(config[CONF_SOMFY_STORAGE_NAMESPACE]))
    cg.add(var.set_initial_rolling_code(config[CONF_INITIAL_ROLLING_CODE]))
    cg.add(var.set_repeat_count(config[CONF_REPEAT_COMMAND_COUNT]))

    if CONF_MY_POSITION in config:
        cg.add(var.set_my_position(config[CONF_MY_POSITION]))
    if CONF_MY_BUTTON in config:
        my_button = await cg.get_variable(config[CONF_MY_BUTTON])
        cg.add(var.set_my_button(my_button))

    if CONF_ENCRYPTION_KEY in config:
        cg.add(var.set_encryption_key(config[CONF_ENCRYPTION_KEY]))

    mode = config[CONF_IOHC_MODE]
    if mode == IOHC_MODE_2W:
        cg.add(var.set_mode(IohcMode.MODE_2W))
        if CONF_TARGET_NODE in config:
            cg.add(var.set_target_node(config[CONF_TARGET_NODE]))
    else:
        cg.add(var.set_mode(IohcMode.MODE_1W))

    # RX state-sync. Always on for 2W (bidirectional by definition); for 1W it
    # mirrors the RTS allowed_remotes/detected_remote opt-in, so plain TX-only
    # 1W covers pull in no extra code or the text_sensor dependency.
    if iohc_uses_rx(config):
        cg.add_define("USE_SOMFY_IOHC_RX")
        for code in config.get(CONF_ALLOWED_REMOTES, []):
            cg.add(var.add_receive_remote_code(code))
        if config.get(CONF_DETECTED_REMOTE):
            log_sensor = await cg.get_variable(config[CONF_DETECTED_REMOTE])
            cg.add(var.set_log_text_sensor(log_sensor))
