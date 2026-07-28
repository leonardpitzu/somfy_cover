"""Tests for RTS-specific config validation."""

import esphome.config_validation as cv
import pytest
from somfy.cover import (
    CONF_ALLOWED_REMOTES,
    CONF_DETECTED_REMOTE,
    CONF_ID,
    CONF_REMOTE_RECEIVER,
    CONF_TYPE,
    DOMAIN,
    TYPE_RTS,
    find_hub_config,
    uses_rx,
    validate_rts_config,
)

RX_HUB = {CONF_ID: "rts_radio", CONF_REMOTE_RECEIVER: "receiver_id"}
TX_ONLY_HUB = {CONF_ID: "rts_radio"}


# ---------------------------------------------------------------------------
# uses_rx()
# ---------------------------------------------------------------------------

class TestUsesRx:
    """Verify uses_rx correctly detects whether an RX path is configured."""

    @pytest.mark.parametrize("config,expected", [
        # Positive cases
        ({CONF_DETECTED_REMOTE: "sensor_id", CONF_ALLOWED_REMOTES: []}, True),
        ({CONF_DETECTED_REMOTE: "sensor_id", CONF_ALLOWED_REMOTES: [0x112233]}, True),
        ({CONF_ALLOWED_REMOTES: [0x112233]}, True),
        ({CONF_ALLOWED_REMOTES: [0xABCDEF, 0x112233]}, True),
        # Negative cases
        ({CONF_ALLOWED_REMOTES: []}, False),
        ({}, False),
    ], ids=[
        "detected-only",
        "detected-and-allowed",
        "allowed-single",
        "allowed-multiple",
        "empty-allowed",
        "empty-config",
    ])
    def test_detection(self, config, expected):
        assert uses_rx(config) is expected

    def test_detected_remote_none_is_falsy(self):
        """None value for detected_remote should be treated as absent."""
        config = {CONF_DETECTED_REMOTE: None, CONF_ALLOWED_REMOTES: []}
        assert uses_rx(config) is False

    def test_detected_remote_empty_string_is_falsy(self):
        config = {CONF_DETECTED_REMOTE: "", CONF_ALLOWED_REMOTES: []}
        assert uses_rx(config) is False

    def test_does_not_mutate_config(self):
        config = {CONF_ALLOWED_REMOTES: [0xABCDEF]}
        original = dict(config)
        uses_rx(config)
        assert config == original


# ---------------------------------------------------------------------------
# find_hub_config()
# ---------------------------------------------------------------------------

class TestFindHubConfig:
    """The cover must locate the hub it references in the full config."""

    def test_finds_matching_hub(self):
        full = {DOMAIN: [{CONF_ID: "other"}, RX_HUB]}
        assert find_hub_config(full, "rts_radio") is RX_HUB

    def test_compares_by_string_not_identity(self):
        """Hub ids are ID objects; equality must go through str()."""

        class FakeId:
            def __init__(self, name):
                self.name = name

            def __str__(self):
                return self.name

        hub = {CONF_ID: FakeId("rts_radio")}
        full = {DOMAIN: [hub]}
        assert find_hub_config(full, FakeId("rts_radio")) is hub

    def test_returns_none_when_absent(self):
        full = {DOMAIN: [{CONF_ID: "other"}]}
        assert find_hub_config(full, "rts_radio") is None

    @pytest.mark.parametrize("full", [{}, {DOMAIN: None}, {DOMAIN: []}],
                             ids=["missing-key", "none-value", "empty-list"])
    def test_tolerates_missing_domain(self, full):
        assert find_hub_config(full, "rts_radio") is None

    def test_accepts_single_dict(self):
        full = {DOMAIN: RX_HUB}
        assert find_hub_config(full, "rts_radio") is RX_HUB


# ---------------------------------------------------------------------------
# validate_rts_config() — happy paths
# ---------------------------------------------------------------------------

class TestValidateRtsConfigValid:
    """Valid configurations must pass without raising."""

    @pytest.mark.parametrize("config,hub", [
        # Full RX path: receiver + allowed + detected
        (
            {
                CONF_TYPE: TYPE_RTS,
                CONF_DETECTED_REMOTE: "sensor_id",
                CONF_ALLOWED_REMOTES: [0xABCDEF],
            },
            RX_HUB,
        ),
        # Receiver + allowed remotes (no detected)
        (
            {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: [0x123456]},
            RX_HUB,
        ),
        # Receiver + detected remote (empty allowed)
        (
            {
                CONF_TYPE: TYPE_RTS,
                CONF_DETECTED_REMOTE: "sensor_id",
                CONF_ALLOWED_REMOTES: [],
            },
            RX_HUB,
        ),
        # Hub has a receiver but the cover uses no RX feature
        (
            {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: []},
            RX_HUB,
        ),
        # TX-only: no receiver on the hub, no RX features on the cover
        (
            {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: []},
            TX_ONLY_HUB,
        ),
        # Hub could not be resolved -> skip rather than guess
        (
            {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: [0x123456]},
            None,
        ),
    ], ids=[
        "full-rx-path",
        "receiver-and-allowed",
        "receiver-and-detected",
        "receiver-unused",
        "tx-only",
        "unresolved-hub",
    ])
    def test_passes(self, config, hub):
        assert validate_rts_config(config, hub) is config

    def test_does_not_mutate(self):
        config = {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: [0xAA]}
        original_keys = set(config.keys())
        validate_rts_config(config, RX_HUB)
        assert set(config.keys()) == original_keys


# ---------------------------------------------------------------------------
# validate_rts_config() — invalid configurations
# ---------------------------------------------------------------------------

class TestValidateRtsConfigInvalid:
    """RX features on a cover whose hub has no receiver must raise cv.Invalid."""

    @pytest.mark.parametrize("config", [
        {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: [0x123456]},
        {CONF_TYPE: TYPE_RTS, CONF_DETECTED_REMOTE: "sensor_id", CONF_ALLOWED_REMOTES: []},
        {
            CONF_TYPE: TYPE_RTS,
            CONF_DETECTED_REMOTE: "sensor_id",
            CONF_ALLOWED_REMOTES: [0xABCDEF],
        },
        {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: [0x111111, 0x222222, 0x333333]},
    ], ids=["allowed-no-rx", "detected-no-rx", "both-no-rx", "multiple-allowed-no-rx"])
    def test_raises_invalid(self, config):
        with pytest.raises(cv.Invalid):
            validate_rts_config(config, TX_ONLY_HUB)


# ---------------------------------------------------------------------------
# Error message quality
# ---------------------------------------------------------------------------

class TestValidateRtsErrorMessages:
    """Error messages must guide the user to the fix."""

    def test_mentions_remote_receiver(self):
        config = {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: [0x123456]}
        with pytest.raises(cv.Invalid, match=CONF_REMOTE_RECEIVER):
            validate_rts_config(config, TX_ONLY_HUB)

    def test_mentions_allowed_remotes_or_detected_remote(self):
        config = {CONF_TYPE: TYPE_RTS, CONF_ALLOWED_REMOTES: [0x123456]}
        with pytest.raises(cv.Invalid, match=f"{CONF_ALLOWED_REMOTES}|{CONF_DETECTED_REMOTE}"):
            validate_rts_config(config, TX_ONLY_HUB)
