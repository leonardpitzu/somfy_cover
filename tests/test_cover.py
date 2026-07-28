"""Protocol-agnostic tests for the somfy cover component.

Focuses on structural integrity, cross-module consistency, and schema
composition rather than trivial string equality.
"""

from typing import ClassVar

import pytest
import somfy as hub_mod
import somfy.cover as cover_mod
from somfy.cover import (
    CODEOWNERS,
    COMMON_COVER_FIELDS,
    DEPENDENCIES,
    TYPE_IOHC,
    TYPE_RTS,
)

# ---------------------------------------------------------------------------
# Cross-module consistency
# ---------------------------------------------------------------------------

class TestCrossModuleConsistency:
    """Hub and cover modules must agree on shared symbols."""

    def test_type_rts_matches(self):
        assert cover_mod.TYPE_RTS == hub_mod.TYPE_RTS

    def test_type_iohc_matches(self):
        assert cover_mod.TYPE_IOHC == hub_mod.TYPE_IOHC

    def test_types_are_distinct(self):
        assert TYPE_RTS != TYPE_IOHC

    def test_remote_receiver_key_matches(self):
        assert cover_mod.CONF_REMOTE_RECEIVER == hub_mod.CONF_REMOTE_RECEIVER


# ---------------------------------------------------------------------------
# COMMON_COVER_FIELDS structure
# ---------------------------------------------------------------------------

class TestCommonCoverFields:
    """Verify COMMON_COVER_FIELDS structural properties."""

    def test_is_dict(self):
        assert isinstance(COMMON_COVER_FIELDS, dict)

    def test_is_non_empty(self):
        assert len(COMMON_COVER_FIELDS) > 0

    def test_no_protocol_specific_keys_leak_in(self):
        """COMMON_COVER_FIELDS must not contain iohc/rts-only keys."""
        field_strs = " ".join(str(k) for k in COMMON_COVER_FIELDS)
        for forbidden in ("allowed_remotes", "detected_remote", "target_node", "mode"):
            assert forbidden not in field_strs


# ---------------------------------------------------------------------------
# Module metadata
# ---------------------------------------------------------------------------

class TestModuleMetadata:
    """Verify module-level metadata is sane."""

    def test_codeowners_is_list(self):
        assert isinstance(CODEOWNERS, list)

    def test_codeowners_not_empty(self):
        assert len(CODEOWNERS) > 0

    def test_codeowners_entries_start_with_at(self):
        assert all(owner.startswith("@") for owner in CODEOWNERS)

    def test_dependencies_is_list(self):
        assert isinstance(DEPENDENCIES, list)

    def test_dependencies_include_esp32(self):
        assert "esp32" in DEPENDENCIES

    def test_hub_multi_conf_enabled(self):
        assert hub_mod.MULTI_CONF is True

    def test_hub_auto_load_includes_button(self):
        assert "button" in hub_mod.AUTO_LOAD


# ---------------------------------------------------------------------------
# Module importability (regression guard)
# ---------------------------------------------------------------------------

class TestImports:
    """All public symbols must be importable — catches accidental renames."""

    EXPECTED_COVER_SYMBOLS: ClassVar[list[str]] = [
        "TYPE_RTS", "TYPE_IOHC",
        "CONF_SOMFY_ID", "CONF_REMOTE_CODE", "CONF_ENCRYPTION_KEY",
        "CONF_IOHC_MODE", "CONF_TARGET_NODE",
        "CONF_ALLOWED_REMOTES", "CONF_DETECTED_REMOTE", "CONF_REMOTE_RECEIVER",
        "CONF_PROG_BUTTON", "CONF_REPEAT_COMMAND_COUNT",
        "CONF_SOMFY_STORAGE_KEY", "CONF_SOMFY_STORAGE_NAMESPACE",
        "IOHC_MODE_1W", "IOHC_MODE_2W",
        "COMMON_COVER_FIELDS",
        "validate_iohc_config", "validate_rts_config", "uses_rx",
        "find_hub_config", "FINAL_VALIDATE_SCHEMA",
    ]

    EXPECTED_HUB_SYMBOLS: ClassVar[list[str]] = [
        "CONF_REMOTE_TRANSMITTER", "CONF_REMOTE_RECEIVER", "CONF_CC1101_ID",
        "TYPE_RTS", "TYPE_IOHC", "MULTI_CONF", "DOMAIN",
    ]

    @pytest.mark.parametrize("symbol", EXPECTED_COVER_SYMBOLS)
    def test_cover_module_exports(self, symbol):
        assert hasattr(cover_mod, symbol), f"somfy.cover missing '{symbol}'"

    @pytest.mark.parametrize("symbol", EXPECTED_HUB_SYMBOLS)
    def test_hub_module_exports(self, symbol):
        assert hasattr(hub_mod, symbol), f"somfy missing '{symbol}'"
