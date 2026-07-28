"""Tests for iohc 2W (bidirectional) config validation."""

import esphome.config_validation as cv
import pytest
from somfy.cover import (
    CONF_ENCRYPTION_KEY,
    CONF_IOHC_MODE,
    CONF_TARGET_NODE,
    IOHC_MODE_2W,
    validate_iohc_config,
)

# ---------------------------------------------------------------------------
# Happy paths
# ---------------------------------------------------------------------------

class TestIohc2WValidPaths:
    """2W mode must pass when both target_node and encryption_key are set."""

    def test_passes_with_all_required_fields(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_TARGET_NODE: 0x123456,
            CONF_ENCRYPTION_KEY: "34C3466ED88F4E8E16AA473949884373",
        }
        assert validate_iohc_config(config) is config

    def test_returns_same_config_object(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_TARGET_NODE: 0xABCDEF,
            CONF_ENCRYPTION_KEY: "FF" * 16,
        }
        assert validate_iohc_config(config) is config

    def test_does_not_mutate_config(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_TARGET_NODE: 0x111111,
            CONF_ENCRYPTION_KEY: "AA" * 16,
        }
        original_keys = set(config.keys())
        validate_iohc_config(config)
        assert set(config.keys()) == original_keys

    @pytest.mark.parametrize("target_node", [0x000001, 0x7FFFFF, 0xFFFFFF])
    def test_various_target_node_values(self, target_node):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_TARGET_NODE: target_node,
            CONF_ENCRYPTION_KEY: "00" * 16,
        }
        assert validate_iohc_config(config) is config


# ---------------------------------------------------------------------------
# Missing required fields
# ---------------------------------------------------------------------------

class TestIohc2WMissingFields:
    """2W mode must raise cv.Invalid when required fields are absent."""

    def test_fails_without_target_node(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_ENCRYPTION_KEY: "34C3466ED88F4E8E16AA473949884373",
        }
        with pytest.raises(cv.Invalid, match=CONF_TARGET_NODE):
            validate_iohc_config(config)

    def test_fails_without_encryption_key(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_TARGET_NODE: 0x123456,
        }
        with pytest.raises(cv.Invalid, match=CONF_ENCRYPTION_KEY):
            validate_iohc_config(config)

    def test_fails_without_both_required_fields(self):
        config = {CONF_IOHC_MODE: IOHC_MODE_2W}
        with pytest.raises(cv.Invalid):
            validate_iohc_config(config)

    def test_target_node_none_treated_as_missing(self):
        """An explicit None should still fail (key present but falsy)."""
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_ENCRYPTION_KEY: "BB" * 16,
            # CONF_TARGET_NODE intentionally absent
        }
        with pytest.raises(cv.Invalid):
            validate_iohc_config(config)


# ---------------------------------------------------------------------------
# Error message quality
# ---------------------------------------------------------------------------

class TestIohc2WErrorMessages:
    """Error messages must mention the relevant field names."""

    def test_missing_target_node_mentions_field(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_ENCRYPTION_KEY: "CC" * 16,
        }
        with pytest.raises(cv.Invalid, match="target_node"):
            validate_iohc_config(config)

    def test_missing_encryption_key_mentions_field(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_TARGET_NODE: 0x123456,
        }
        with pytest.raises(cv.Invalid, match="encryption_key"):
            validate_iohc_config(config)

    def test_missing_target_node_mentions_2w(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_ENCRYPTION_KEY: "DD" * 16,
        }
        with pytest.raises(cv.Invalid, match="2w|2W"):
            validate_iohc_config(config)
