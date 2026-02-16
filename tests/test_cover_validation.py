"""Tests for the ESPHome config-validation helpers in cover.py."""

import pytest

from somfy_cover.cover import (
    CONF_ALLOWED_REMOTES,
    CONF_DETECTED_REMOTE,
    CONF_REMOTE_RECEIVER,
    uses_rx,
    validate_rx_config,
)

import esphome.config_validation as cv


# ---------------------------------------------------------------------------
# uses_rx()
# ---------------------------------------------------------------------------

class TestUsesRx:
    """Verify uses_rx correctly detects whether an RX path is configured."""

    def test_true_when_remote_receiver_present(self):
        config = {
            CONF_REMOTE_RECEIVER: "receiver_id",
            CONF_ALLOWED_REMOTES: [],
        }
        assert uses_rx(config) is True

    def test_true_when_detected_remote_present(self):
        config = {
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [],
        }
        assert uses_rx(config) is True

    def test_true_when_allowed_remotes_non_empty(self):
        config = {
            CONF_ALLOWED_REMOTES: [0x112233],
        }
        assert uses_rx(config) is True

    def test_true_when_all_rx_fields_set(self):
        config = {
            CONF_REMOTE_RECEIVER: "receiver_id",
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [0xABCDEF, 0x112233],
        }
        assert uses_rx(config) is True

    def test_false_when_no_rx_config(self):
        config = {
            CONF_ALLOWED_REMOTES: [],
        }
        assert uses_rx(config) is False

    def test_false_when_allowed_remotes_empty_and_no_others(self):
        config = {
            CONF_ALLOWED_REMOTES: [],
        }
        assert uses_rx(config) is False


# ---------------------------------------------------------------------------
# validate_rx_config()
# ---------------------------------------------------------------------------

class TestValidateRxConfig:
    """Verify validate_rx_config raises on invalid RX combinations."""

    def test_passes_with_all_rx_fields(self):
        config = {
            CONF_REMOTE_RECEIVER: "receiver_id",
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [0xABCDEF],
        }
        assert validate_rx_config(config) is config

    def test_passes_with_receiver_and_allowed_remotes(self):
        config = {
            CONF_REMOTE_RECEIVER: "receiver_id",
            CONF_ALLOWED_REMOTES: [0x123456],
        }
        assert validate_rx_config(config) is config

    def test_passes_with_receiver_and_detected_remote(self):
        config = {
            CONF_REMOTE_RECEIVER: "receiver_id",
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [],
        }
        assert validate_rx_config(config) is config

    def test_passes_with_receiver_only(self):
        config = {
            CONF_REMOTE_RECEIVER: "receiver_id",
            CONF_ALLOWED_REMOTES: [],
        }
        assert validate_rx_config(config) is config

    def test_passes_without_any_rx_config(self):
        config = {
            CONF_ALLOWED_REMOTES: [],
        }
        assert validate_rx_config(config) is config

    def test_fails_when_allowed_remotes_without_receiver(self):
        config = {
            CONF_ALLOWED_REMOTES: [0x123456],
        }
        with pytest.raises(cv.Invalid):
            validate_rx_config(config)

    def test_fails_when_detected_remote_without_receiver(self):
        config = {
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [],
        }
        with pytest.raises(cv.Invalid):
            validate_rx_config(config)

    def test_fails_when_both_set_without_receiver(self):
        config = {
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [0xABCDEF],
        }
        with pytest.raises(cv.Invalid):
            validate_rx_config(config)

    def test_error_message_mentions_remote_receiver(self):
        config = {
            CONF_ALLOWED_REMOTES: [0x123456],
        }
        with pytest.raises(cv.Invalid, match=CONF_REMOTE_RECEIVER):
            validate_rx_config(config)


# ---------------------------------------------------------------------------
# Constants sanity checks
# ---------------------------------------------------------------------------

class TestConstants:
    """Verify key constants are defined correctly (catches typos/renames)."""

    def test_conf_remote_receiver_value(self):
        assert CONF_REMOTE_RECEIVER == "remote_receiver"

    def test_conf_allowed_remotes_value(self):
        assert CONF_ALLOWED_REMOTES == "allowed_remotes"

    def test_conf_detected_remote_value(self):
        assert CONF_DETECTED_REMOTE == "detected_remote"

    def test_codeowners(self):
        from somfy_cover.cover import CODEOWNERS
        assert "@LeonardPitzu" in CODEOWNERS

    def test_dependencies_include_esp32(self):
        from somfy_cover.cover import DEPENDENCIES
        assert "esp32" in DEPENDENCIES
