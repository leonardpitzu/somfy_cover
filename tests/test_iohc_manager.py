"""Validation tests for the runtime IOHC commissioning manager."""

import importlib.util
from pathlib import Path

import pytest
import esphome.config_validation as cv


def _load_manager_module():
    path = (
        Path(__file__).parent.parent
        / "components"
        / "somfy_iohc_manager"
        / "__init__.py"
    )
    spec = importlib.util.spec_from_file_location("somfy_iohc_manager", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


manager = _load_manager_module()


def _manager_cpp() -> str:
    return (
        Path(__file__).parent.parent
        / "components"
        / "somfy_iohc_manager"
        / "somfy_iohc_manager.cpp"
    ).read_text()


@pytest.mark.parametrize(
    "key",
    ["0" * 31, "0" * 33, "Z" * 32, "00112233445566778899AABBCCDDEEFG"],
)
def test_backup_key_requires_exact_hex(key):
    with pytest.raises(cv.Invalid, match="32 hexadecimal"):
        manager.validate_hex_key(key)


def test_valid_backup_key_is_preserved():
    key = "FFEEDDCCBBAA99887766554433221100"
    assert manager.validate_hex_key(key) == key


def test_import_slots_must_be_unique():
    config = {
        manager.CONF_MAX_SHUTTERS: 4,
        manager.CONF_IMPORTS: [
            {manager.CONF_SLOT: 1},
            {manager.CONF_SLOT: 1},
        ],
    }
    with pytest.raises(cv.Invalid, match="must be unique"):
        manager.validate_manager(config)


def test_import_slot_must_fit_pool():
    config = {
        manager.CONF_MAX_SHUTTERS: 2,
        manager.CONF_IMPORTS: [{manager.CONF_SLOT: 2}],
    }
    with pytest.raises(cv.Invalid, match="outside max_shutters"):
        manager.validate_manager(config)


def test_pairing_uncertainty_is_persisted_before_rf():
    source = _manager_cpp()
    function = source.split("void SomfyIohcManager::transmit_pairing_", 1)[1]
    function = function.split("void SomfyIohcManager::confirm_pairing_", 1)[0]
    assert function.index("ManagedSlotState::PAIR_SENT") < function.index(
        "runtime_program()"
    )


def test_exhausted_rolling_code_backup_cannot_be_restored():
    source = _manager_cpp()
    function = source.split("void SomfyIohcManager::restore_service", 1)[1]
    function = function.split("uint32_t SomfyIohcManager::record_checksum_", 1)[0]
    assert "rolling_code_exhausted" in function


def test_slot_move_disables_and_persists_source_before_destination():
    source = _manager_cpp()
    function = source.split("void SomfyIohcManager::move_service", 1)[1]
    function = function.split("bool SomfyIohcManager::open_registry_", 1)[0]
    assert function.index("ManagedSlotState::ARCHIVED") < function.index(
        "complete_move_"
    )
    assert "runtime_program" not in function


def test_yaml_imports_are_one_time_and_duplicate_aware():
    source = _manager_cpp()
    setup = source.split("void SomfyIohcManager::setup()", 1)[1]
    setup = setup.split("void SomfyIohcManager::loop()", 1)[0]
    assert "read_imports_bootstrapped_" in setup
    assert "node_id_in_use_" in setup
    assert "mark_imports_bootstrapped_" in setup


def test_slot_swap_journals_both_identities_before_writing_destinations():
    source = _manager_cpp()
    service = source.split("void SomfyIohcManager::swap_service", 1)[1]
    service = service.split("bool SomfyIohcManager::open_registry_", 1)[0]
    assert service.index("save_swap_journal_") < service.index("complete_swap_")
    assert "runtime_program" not in service

    completion = source.split("bool SomfyIohcManager::complete_swap_", 1)[1]
    completion = completion.split("bool SomfyIohcManager::recover_pending_swap_", 1)[0]
    assert completion.count("save_record_") == 2
    assert completion.index("save_record_") < completion.index("erase_swap_journal_")
    assert completion.rindex("save_record_") < completion.index("erase_swap_journal_")


def test_slot_swap_is_registered_as_a_local_api_action():
    source = _manager_cpp()
    assert '"somfy_swap", {"slot", "target_slot"}' in source


def test_group_remote_aliases_are_many_to_many_and_receive_only():
    root = Path(__file__).parent.parent
    source = _manager_cpp()
    header = (
        root
        / "components"
        / "somfy_iohc_manager"
        / "somfy_iohc_manager.h"
    ).read_text()

    assert "IOHC_MANAGER_MAX_REMOTE_ALIASES = 32" in header
    assert "node_ids[IOHC_MANAGER_MAX_SHUTTERS]" in header
    assert '"somfy_remote_alias"' in source
    service = source.split("void SomfyIohcManager::remote_alias_service", 1)[1]
    service = service.split("bool SomfyIohcManager::parse_remote_code_", 1)[0]
    assert 'action == "discover"' in service
    assert 'action == "set"' in service
    assert 'action == "remove"' in service
    assert "runtime_program" not in service


def test_group_remote_membership_uses_permanent_nodes_not_slot_positions():
    source = _manager_cpp()
    resolver = source.split("bool SomfyIohcManager::resolve_alias_slots_", 1)[1]
    resolver = resolver.split("std::string SomfyIohcManager::alias_slots_for_nodes_", 1)[0]
    assert "managed.record.node_id" in resolver
    apply_filter = source.split(
        "void SomfyIohcManager::apply_slot_remote_filters_", 1
    )[1]
    apply_filter = apply_filter.split(
        "bool SomfyIohcManager::complete_move_", 1
    )[0]
    assert "record.node_id" in apply_filter
    assert "alias.record.remote_code" in apply_filter


def test_group_remote_capture_requires_a_user_movement_command():
    source = _manager_cpp()
    receiver = source.split("void SomfyIohcManager::on_iohc_packet_", 1)[1]
    assert "alias_discovery_active_" in receiver
    assert "packet.dest_node == iohc::BROADCAST_ADDR" in receiver
    assert "main_param == iohc_cmd::MP_OPEN" in receiver
    assert "main_param == iohc_cmd::MP_CLOSE" in receiver
    assert '"alias_remote_detected"' in receiver


def test_remote_events_report_the_actual_group_source_and_rssi():
    root = Path(__file__).parent.parent
    source = _manager_cpp()
    cover_header = (root / "components" / "somfy" / "somfy_iohc.h").read_text()
    cover_source = (root / "components" / "somfy" / "somfy_iohc.cpp").read_text()
    assert "std::function<void(uint16_t, uint32_t, float)>" in cover_header
    assert "event_code, pkt.src_node, pkt.rssi" in cover_source
    assert "main_param, pkt.src_node, pkt.rssi" in cover_source
    assert 'publish_status_("remote_command", index, detail, rssi, remote)' in source


def test_manager_exposes_versioned_status_without_native_ui_buttons():
    root = Path(__file__).parent.parent
    manager_python = (
        root / "components" / "somfy_iohc_manager" / "__init__.py"
    ).read_text()
    manager_header = (
        root / "components" / "somfy_iohc_manager" / "somfy_iohc_manager.h"
    ).read_text()
    source = _manager_cpp()
    assert r'{\"v\":1' in source
    assert 'AUTO_LOAD = ["cover", "text_sensor"]' in manager_python
    assert 'CORE.register_platform_component("button", var)' not in manager_python
    assert "App.register_button" not in source
    assert "SomfyIohcMyButton" not in manager_header
