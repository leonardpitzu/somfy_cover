"""Tests for iohc 1W (one-way) specific behaviour."""

import pytest
import esphome.config_validation as cv

from somfy.cover import (
    CONF_ENCRYPTION_KEY,
    CONF_IOHC_MODE,
    CONF_MY_BUTTON,
    CONF_MY_POSITION,
    CONF_TARGET_NODE,
    IOHC_MODE_1W,
    validate_iohc_config,
    validate_encryption_key,
)


class TestIohc1WValidation:
    """1W mode should pass validation without target_node or encryption_key."""

    @pytest.mark.parametrize("config", [
        {},
        {CONF_IOHC_MODE: IOHC_MODE_1W},
    ], ids=["default-mode", "explicit-1w"])
    def test_passes_without_extra_fields(self, config):
        assert validate_iohc_config(config) is config

    def test_passes_with_optional_encryption_key(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_1W,
            CONF_ENCRYPTION_KEY: "34C3466ED88F4E8E16AA473949884373",
        }
        assert validate_iohc_config(config) is config

    def test_passes_with_target_node_ignored_in_1w(self):
        """target_node is unused in 1W but shouldn't cause an error."""
        config = {
            CONF_IOHC_MODE: IOHC_MODE_1W,
            CONF_TARGET_NODE: 0xABCDEF,
        }
        assert validate_iohc_config(config) is config

    def test_returns_same_config_object(self):
        """Validator must return the config dict (not a copy) for chaining."""
        config = {CONF_IOHC_MODE: IOHC_MODE_1W}
        result = validate_iohc_config(config)
        assert result is config

    def test_does_not_mutate_config(self):
        """Validator must not add/remove keys."""
        config = {CONF_IOHC_MODE: IOHC_MODE_1W, CONF_ENCRYPTION_KEY: "A" * 32}
        original_keys = set(config.keys())
        validate_iohc_config(config)
        assert set(config.keys()) == original_keys

    def test_my_button_requires_my_position(self):
        with pytest.raises(cv.Invalid, match=CONF_MY_POSITION):
            validate_iohc_config({CONF_MY_BUTTON: "my_button"})

    def test_my_position_without_button_is_valid_for_remote_state_sync(self):
        config = {CONF_MY_POSITION: 0.42}
        assert validate_iohc_config(config) is config

    @pytest.mark.parametrize(
        "key",
        ["0" * 31, "0" * 33, "Z" * 32, "00112233445566778899AABBCCDDEEFG"],
    )
    def test_encryption_key_must_be_exact_hex(self, key):
        with pytest.raises(cv.Invalid, match="32 hexadecimal"):
            validate_encryption_key(key)

    def test_valid_encryption_key_is_preserved(self):
        key = "00112233445566778899AABBCCDDEEFF"
        assert validate_encryption_key(key) == key
