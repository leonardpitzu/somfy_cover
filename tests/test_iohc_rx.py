"""Tests for iohc RX state-sync config (allowed_remotes / detected_remote).

The iohc cover gained feature parity with RTS: it can learn physical
io-homecontrol remote IDs and keep HA in sync when a motor is driven by an
original remote. These tests cover the protocol-agnostic gating helper and that
the iohc validator stays neutral about the RX fields.
"""

import pytest
from somfy.cover import (
    CONF_ALLOWED_REMOTES,
    CONF_DETECTED_REMOTE,
    CONF_ENCRYPTION_KEY,
    CONF_IOHC_MODE,
    CONF_TARGET_NODE,
    IOHC_MODE_1W,
    IOHC_MODE_2W,
    uses_rx,
    validate_iohc_config,
)

# ---------------------------------------------------------------------------
# uses_rx() gates whether RX-sync code is compiled for an iohc cover
# ---------------------------------------------------------------------------

class TestUsesRxForIohc:
    @pytest.mark.parametrize("config,expected", [
        ({CONF_DETECTED_REMOTE: "sensor_id", CONF_ALLOWED_REMOTES: []}, True),
        ({CONF_DETECTED_REMOTE: "sensor_id", CONF_ALLOWED_REMOTES: [0x112233]}, True),
        ({CONF_ALLOWED_REMOTES: [0x112233]}, True),
        ({CONF_ALLOWED_REMOTES: []}, False),
        ({}, False),
    ], ids=[
        "detected-only",
        "detected-and-allowed",
        "allowed-single",
        "empty-allowed",
        "empty-config",
    ])
    def test_detection(self, config, expected):
        assert uses_rx(config) is expected


# ---------------------------------------------------------------------------
# validate_iohc_config() must stay neutral about RX fields
# ---------------------------------------------------------------------------

class TestValidatorIgnoresRxFields:
    def test_1w_with_rx_fields_passes(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_1W,
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [0x112233],
        }
        assert validate_iohc_config(config) is config

    def test_2w_with_rx_fields_passes(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_TARGET_NODE: 0xDEADBE,
            CONF_ENCRYPTION_KEY: "34C3466ED88F4E8E16AA473949884373",
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [0x112233],
        }
        assert validate_iohc_config(config) is config

    def test_rx_fields_do_not_satisfy_2w_requirements(self):
        """RX fields must not be mistaken for the 2W target_node/key requirement."""
        import esphome.config_validation as cv

        config = {
            CONF_IOHC_MODE: IOHC_MODE_2W,
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [0x112233],
        }
        with pytest.raises(cv.Invalid):
            validate_iohc_config(config)

    def test_does_not_mutate_config(self):
        config = {
            CONF_IOHC_MODE: IOHC_MODE_1W,
            CONF_ALLOWED_REMOTES: [0xABCDEF],
            CONF_DETECTED_REMOTE: "sensor_id",
        }
        original_keys = set(config.keys())
        validate_iohc_config(config)
        assert set(config.keys()) == original_keys
