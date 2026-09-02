"""Contract checks for IOHC 1W multi-bridge safety.

These tests intentionally focus on boundaries which must remain true even if
the internal implementation is refactored: one rolling-code owner, a narrow
exact-frame relay allow-list, globally visible receive observations, estimator
updates which cannot transmit, and a fail-disabled persistent transfer.
"""

from pathlib import Path


ROOT = Path(__file__).parent.parent
MANAGER_CPP = (
    ROOT / "components/somfy_iohc_manager/somfy_iohc_manager.cpp"
).read_text()
MANAGER_H = (
    ROOT / "components/somfy_iohc_manager/somfy_iohc_manager.h"
).read_text()
MANAGER_PY = (
    ROOT / "components/somfy_iohc_manager/__init__.py"
).read_text()
COVER_CPP = (ROOT / "components/somfy/somfy_iohc.cpp").read_text()
COVER_H = (ROOT / "components/somfy/somfy_iohc.h").read_text()
HUB_CPP = (ROOT / "components/somfy/somfy_hub_iohc.cpp").read_text()
STORAGE_CPP = (ROOT / "components/somfy/NVSRollingCodeStorage.cpp").read_text()
STORAGE_H = (ROOT / "components/somfy/NVSRollingCodeStorage.h").read_text()


def _function(source: str, signature: str, next_signature: str) -> str:
    """Return one C++ function body using neighbouring definitions as bounds."""

    return source.split(signature, 1)[1].split(next_signature, 1)[0]


def _action_branch(function: str, action: str) -> str:
    """Return one action branch without depending on source-code branch order."""

    marker = f'action == "{action}"'
    start = function.index(marker)
    ends = [
        function.find(f'action == "{other}"', start + len(marker))
        for other in (
            "query",
            "prepare",
            "import",
            "commit",
            "activate",
            "finalize",
            "abort",
            "rollback",
        )
        if other != action
    ]
    ends = [end for end in ends if end >= 0]
    return function[start : min(ends) if ends else len(function)]


def test_exact_additive_native_api_contract_is_registered():
    assert (
        '"somfy_redundant_control", {"slot", "command", "relay_token"}'
        in MANAGER_CPP
    )
    assert '"somfy_relay",\n      {"action", "payload"}' in MANAGER_CPP
    assert '"somfy_observe",\n      {"slot", "command", "steps"}' in MANAGER_CPP
    assert (
        '"somfy_transfer",\n      {"action", "slot", "transfer_token", '
        '"encrypted_backup"}'
        in MANAGER_CPP
    )


def test_cluster_relay_key_is_sensitive_and_defaults_without_breaking_yaml():
    assert 'CONF_RELAY_KEY = "relay_key"' in MANAGER_PY
    assert "cv.Optional(CONF_RELAY_KEY)" in MANAGER_PY
    assert "mark_sensitive(validate_hex_key)" in MANAGER_PY
    assert "config.get(CONF_RELAY_KEY, config[CONF_BACKUP_KEY])" in MANAGER_PY
    assert "set_relay_key" in MANAGER_H
    assert "RELAY_GCM_AAD" in MANAGER_CPP


def test_redundant_control_accepts_only_open_close_and_stop():
    function = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::redundant_control_service",
        "void SomfyIohcManager::relay_service",
    )
    for command in ("open", "close", "stop"):
        assert f'command == "{command}"' in function
    for forbidden in ("my", "position", "tilt", "program", "pair"):
        assert f'command == "{forbidden}"' not in function
    assert "relay_capture_active_" in function
    assert "relay_capture_token_" in function
    assert "runtime_my" not in function
    assert "runtime_set_position" not in function
    assert "runtime_set_tilt" not in function
    assert "runtime_program" not in function


def test_primary_offers_canonical_frame_only_after_local_transmit():
    send = _function(
        COVER_CPP,
        "bool SomfyIohcCover::send_1w_command",
        "bool SomfyIohcCover::send_1w_button_event",
    )
    assert send.index("this->hub_->transmit_packet") < send.index(
        "this->relay_frame_callback_"
    )

    callback = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::on_relayable_frame_",
        "bool SomfyIohcManager::validate_relay_frame_",
    )
    assert "relay_capture_active_" in callback
    assert "relay_capture_param_" in callback
    assert "encrypt_relay_envelope_" in callback
    assert 'publish_relay_event_("relay_offer"' in callback


def test_secondary_relay_validator_is_strictly_limited_to_ordinary_frames():
    validator = _function(
        MANAGER_CPP,
        "bool SomfyIohcManager::validate_relay_frame_",
        "bool SomfyIohcManager::encrypt_relay_envelope_",
    )
    # Ordinary 1W EXECUTE layout: F6 00 dst[3] src[3] cmd data[6]
    # seq[2] mac[6] crc[2]. No other authenticated frame class is relayable.
    assert "frame.size() != 25" in validator
    assert "frame[0] != 0xF6" in validator
    assert "frame[1] != 0x00" in validator
    assert "frame[2] != 0x00" in validator
    assert "frame[3] != 0x00" in validator
    assert "frame[4] != 0x3F" in validator
    assert "frame[8] != iohc_cmd::CMD_EXECUTE" in validator
    assert "frame[9] != iohc_cmd::ORIGINATOR_USER" in validator
    assert "frame[10] != iohc_cmd::ACEI_DEFAULT" in validator
    assert "frame[13] != 0x00" in validator
    assert "frame[14] != 0x00" in validator
    assert "crc16_kermit(frame.data(), frame.size()) != 0" in validator
    for parameter in ("MP_OPEN", "MP_CLOSE", "MP_STOP"):
        assert f"iohc_cmd::{parameter}" in validator
    assert "MP_MY" not in validator
    assert "CMD_BUTTON_EVENT" not in validator
    assert "CMD_WRITE_PRIVATE" not in validator
    assert "CMD_REMOVE_CONTROLLER" not in validator


def test_relay_arm_is_single_use_and_does_not_touch_rolling_storage():
    relay = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::relay_service",
        "void SomfyIohcManager::observe_service",
    )
    assert 'action == "arm"' in relay
    assert 'action == "cancel"' in relay
    assert 'action != "send"' in relay
    assert "decrypt_relay_envelope_" in relay
    assert "validate_relay_frame_" in relay
    assert relay.index("relay_armed_ = false") < relay.index(
        "hub_->transmit_packet"
    )
    assert 'publish_relay_event_("relay_sent"' in relay
    for forbidden in (
        "next_rolling_code_",
        "nextCode",
        "build_1w_frame",
        "runtime_program",
    ):
        assert forbidden not in relay


def test_manager_decodes_globally_even_when_it_has_no_matching_slot():
    setup = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::setup()",
        "void SomfyIohcManager::loop()",
    )
    assert "hub_->register_rx_callback" in setup
    assert "on_iohc_packet_" in setup

    receiver = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::on_iohc_packet_",
        "void SomfyIohcManager::on_relayable_frame_",
    )
    assert "emit_observation_" in receiver
    # Observation emission cannot be conditional on at least one local slot.
    emitter = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::emit_observation_",
        "bool SomfyIohcManager::observed_frame_duplicate_",
    )
    assert "slots_for_remote_" in emitter
    assert "publish_remote_observation_" in emitter
    assert "if (slot_mask" not in emitter.split("publish_remote_observation_", 1)[0]

    # Manager-owned covers suppress their legacy per-cover decoder so one RF
    # frame cannot advance a local estimator twice.
    assert "set_manager_rx_owned(true)" in MANAGER_CPP
    assert "manager_rx_owned_" in COVER_CPP


def test_normalized_observation_status_has_cross_bridge_dedup_fields():
    publisher = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::publish_remote_observation_",
        "void SomfyIohcManager::publish_relay_event_",
    )
    for field in (
        r'\"observation_v\":1',
        r'\"remote\"',
        r'\"command\"',
        r'\"sequence\"',
        r'\"has_sequence\"',
        r'\"complete\"',
        r'\"steps\"',
        r'\"rssi\"',
        r'\"slot_mask\"',
    ):
        assert field in publisher
    assert 'publish_state(output)' in publisher


def test_prefix_and_terminal_observations_keep_distinct_evidence_quality():
    receiver = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::on_iohc_packet_",
        "void SomfyIohcManager::stage_observed_stop_",
    )
    # STOP/MY and tilt terminal events both require the same-remote adjacent
    # sequence helper before consuming a staged D200 prefix.
    assert receiver.count("iohc_proto::gesture_terminal_matches_prefix(") == 2
    assert "this->pending_observed_stop_.sequence" in receiver
    assert "sequence, has_sequence" in receiver

    stop_terminal = receiver.split(
        "if (action == iohc_cmd::BUTTON_ACTION_STOP_MY", 1
    )[1].split(
        "if (action == iohc_cmd::BUTTON_ACTION_TILT_CLOCKWISE", 1
    )[0]
    assert "this->pending_observed_stop_ = {};" in stop_terminal
    assert 'emit_observation_("stop_my", packet.src_node' in stop_terminal
    assert "sequence, has_sequence, 1, true" in stop_terminal

    tilt_terminal = receiver.split(
        "if (action == iohc_cmd::BUTTON_ACTION_TILT_CLOCKWISE", 1
    )[1].split("if (packet.cmd != iohc_cmd::CMD_EXECUTE", 1)[0]
    assert "steps = this->pending_observed_stop_.steps" in tilt_terminal
    assert "packet.src_node, strongest_rssi, sequence, has_sequence, steps," in tilt_terminal
    assert "true);" in tilt_terminal

    timeout = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::flush_observed_stop_",
        "bool SomfyIohcManager::observed_frame_duplicate_",
    )
    assert 'emit_observation_("stop_my", pending.remote' in timeout
    assert "pending.sequence, pending.has_sequence, 1, false" in timeout

    header_contract = MANAGER_H.split("void publish_remote_observation_", 1)[1]
    header_contract = header_contract.split("void publish_relay_event_", 1)[0]
    assert "bool complete" in header_contract


def test_estimator_forwarding_has_no_rf_or_rolling_code_path():
    service = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::observe_service",
        "void SomfyIohcManager::transfer_service",
    )
    for command in (
        "open",
        "close",
        "stop_my",
        "tilt_clockwise",
        "tilt_counterclockwise",
    ):
        assert f'"{command}"' in service or f'"{command}"' in MANAGER_CPP
    assert "apply_observation_to_slot_" in service
    assert 'publish_status_("observation_applied"' in service
    for forbidden in (
        "transmit_packet",
        "runtime_open",
        "runtime_close",
        "runtime_stop()",
        "runtime_my",
        "runtime_program",
        "nextCode",
    ):
        assert forbidden not in service

    apply = _function(
        MANAGER_CPP,
        "bool SomfyIohcManager::apply_observation_to_slot_",
        "bool SomfyIohcManager::encrypt_relay_envelope_",
    )
    assert "runtime_observe_command" in apply
    assert "runtime_observe_tilt" in apply
    for forbidden in ("runtime_open", "runtime_close", "transmit_packet"):
        assert forbidden not in apply

    assert "runtime_observe_command" in COVER_H
    assert "runtime_observe_tilt" in COVER_H
    observed = _function(
        COVER_CPP,
        "void SomfyIohcCover::runtime_observe_command",
        "void SomfyIohcCover::runtime_observe_tilt",
    )
    assert "handle_rx_command_" in observed
    assert "send_1w" not in observed


def test_transfer_uses_a_persistent_fail_disabled_journal():
    assert "struct __attribute__((packed)) ManagedTransferJournal" in MANAGER_H
    assert "ManagedTransferRole" in MANAGER_H
    assert 'TRANSFER_JOURNAL_KEY[] = "xfer_v1"' in MANAGER_CPP
    assert "save_transfer_journal_" in MANAGER_CPP
    assert "read_transfer_journal_" in MANAGER_CPP
    assert "recover_pending_transfer_" in MANAGER_CPP

    setup = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::setup()",
        "void SomfyIohcManager::loop()",
    )
    assert "recover_pending_transfer_" in setup


def test_transfer_actions_preserve_exactly_one_active_owner():
    transfer = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::transfer_service",
        "bool SomfyIohcManager::open_registry_",
    )
    for action in (
        "query",
        "prepare",
        "import",
        "commit",
        "activate",
        "finalize",
        "abort",
        "rollback",
    ):
        assert f'action == "{action}"' in transfer
    for status in (
        "transfer_prepared",
        "transfer_imported",
        "transfer_committed",
        "transfer_activated",
        "transfer_finalized",
        "transfer_aborted",
        "transfer_rolled_back",
    ):
        assert f'"{status}"' in transfer
    assert "ManagedSlotState::ARCHIVED" in transfer
    assert "ManagedSlotState::ACTIVE" in transfer
    assert "seed_slot_rolling_code_" in transfer
    assert "runtime_program" not in transfer

    # Preparing persists ARCHIVED and the journal before it can publish an
    # export; activation cannot happen in the import branch.
    prepare = _action_branch(transfer, "prepare")
    fresh_prepare = prepare.rsplit("auto &managed = this->slots_[slot];", 1)[1]
    assert fresh_prepare.index("ManagedSlotState::ARCHIVED") < fresh_prepare.index(
        "save_record_"
    )
    assert fresh_prepare.index("save_transfer_journal_") < fresh_prepare.index(
        '"transfer_prepared"'
    )

    imported = _action_branch(transfer, "import")
    assert "ManagedSlotState::ARCHIVED" in imported
    assert "ManagedSlotState::ACTIVE" in imported
    assert "seed_slot_rolling_code_" in imported
    assert '"transfer_imported"' in imported
    # A repeated import is a convergence/readback operation. If activation was
    # already durable, copy the journaled ACTIVE record unchanged and re-emit
    # activated rather than demoting the only live owner to ARCHIVED.
    resumed_import = imported.split("if (found &&", 1)[1].split(
        "if (found ||", 1
    )[0]
    assert "managed.record = journal.record" in resumed_import
    assert "managed.record.state =" not in resumed_import
    assert "journal.record.state" in resumed_import
    assert 'active ? "transfer_activated"' in resumed_import
    assert ': "transfer_imported"' in resumed_import

    commit = _action_branch(transfer, "commit")
    assert "erase_record_" in commit
    assert '"transfer_committed"' in commit

    activate = _action_branch(transfer, "activate")
    assert "ManagedSlotState::ACTIVE" in activate
    assert "journal.record = managed.record" in activate
    assert "save_transfer_journal_(journal)" in activate
    assert "erase_transfer_journal_" not in activate
    assert '"transfer_activated"' in activate

    finalize = _action_branch(transfer, "finalize")
    assert "ManagedTransferRole::DESTINATION" in finalize
    assert "journal.record.state" in finalize
    assert "ManagedSlotState::ACTIVE" in finalize
    assert "erase_transfer_journal_" in finalize
    assert "erase_record_" not in finalize
    assert '"transfer_finalized"' in finalize


def test_transfer_import_force_seeds_destination_nvs_before_activation():
    assert "seedNextCode(uint16_t code)" in STORAGE_H
    seed = _function(
        STORAGE_CPP,
        "bool NVSRollingCodeStorage::seedNextCode",
        "uint16_t NVSRollingCodeStorage::nextCode",
    )
    assert "code == 0" in seed
    assert "nvs_set_u16" in seed
    assert "nvs_commit" in seed

    seed_slot = _function(
        MANAGER_CPP,
        "bool SomfyIohcManager::seed_slot_rolling_code_",
        "void SomfyIohcManager::apply_slot_remote_filters_",
    )
    assert "managed.record.storage_namespace" in seed_slot
    assert "managed.record.storage_key" in seed_slot
    assert "NVSRollingCodeStorage storage(" in seed_slot
    assert "storage.seedNextCode(next_code)" in seed_slot
    assert "managed.cover" not in seed_slot


def test_manager_advertises_multi_bridge_capabilities_additively():
    publisher = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::publish_status_",
        "void SomfyIohcManager::stage_remote_command_",
    )
    assert r'{\"v\":1' in publisher
    assert r'\"multi_bridge\":true' in publisher
    # Compact bit mask stays within ESPHome's 255-byte text-state transport:
    # observation=1, estimator apply=2, exact relay=4, transfer=8.
    assert r'\"mb_caps\":15' in publisher


def test_archiving_hard_quiesces_every_delayed_transmit_path():
    disable = _function(
        COVER_CPP,
        "void SomfyIohcCover::set_runtime_enabled",
        "cover::CoverTraits SomfyIohcCover::get_traits",
    )
    assert "if (!enabled)" in disable
    assert "cancel_1w_my_sequence()" in disable
    assert "cancel_1w_tilt_sequence()" in disable
    assert "clear_pending_rx_stop_()" in disable
    assert "rx_sync_.stop()" in disable
    assert "recompute_position_()" in disable
    assert "current_operation = cover::COVER_OPERATION_IDLE" in disable
    assert "stop_prev_trigger_()" in disable
    # The estimator is frozen directly: no normal STOP automation may fire
    # after ownership has moved to another bridge.
    for forbidden in ("runtime_stop", "send_1w_command", "transmit_packet"):
        assert forbidden not in disable

    transfer = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::transfer_service",
        "bool SomfyIohcManager::open_registry_",
    )
    prepare = _action_branch(transfer, "prepare")
    assert prepare.index("set_runtime_enabled(false)") < prepare.index(
        "ManagedSlotState::ARCHIVED"
    )


def test_transfer_query_reports_global_or_slot_state_for_recovery():
    transfer = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::transfer_service",
        "bool SomfyIohcManager::parse_remote_code_",
    )
    query = _action_branch(transfer, "query")
    # Query is deliberately handled before ordinary slot validation so -1 can
    # discover the bridge's one global transfer journal.
    assert transfer.index('action == "query"') < transfer.index(
        "if (!this->valid_slot_(slot))"
    )
    assert "slot >= 0 && slot != journal.slot" in query
    assert 'publish_transfer_state_(journal.slot, journal.token' in query
    assert 'source ? "source" : "destination"' in query
    assert '"archived"' in query
    assert "slot < 0" in query
    assert 'publish_transfer_state_(-1, 0, "none", "none")' in query
    assert "managed.has_record" in query
    assert "state_name_" in query

    publisher = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::publish_transfer_state_",
        "void SomfyIohcManager::stage_remote_command_",
    )
    for field in (
        r'\"action\":\"transfer_state\"',
        r'\"slot\"',
        r'\"transfer_token\"',
        r'\"role\"',
        r'\"phase\"',
    ):
        assert field in publisher


def test_repeated_prepare_and_import_converge_without_new_identity():
    transfer = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::transfer_service",
        "bool SomfyIohcManager::parse_remote_code_",
    )
    prepare = _action_branch(transfer, "prepare")
    assert "if (found)" in prepare
    assert "ManagedTransferRole::SOURCE" in prepare
    assert "journal.slot == slot" in prepare
    assert "set_runtime_enabled(false)" in prepare
    assert "managed.record = journal.record" in prepare
    assert "ManagedSlotState::ARCHIVED" in prepare
    assert '"resumed"' in prepare
    # A resumed prepare must reuse the journal token, not allocate a new
    # controller record or send any RF.
    resumed_prepare = prepare.split("if (found)", 1)[1].split(
        "auto &managed = this->slots_[slot]", 1
    )[0]
    assert "esp_random" not in resumed_prepare
    assert "runtime_program" not in prepare

    imported = _action_branch(transfer, "import")
    assert "ManagedTransferRole::DESTINATION" in imported
    assert "journal.slot == slot && journal.token == token" in imported
    assert "set_runtime_enabled(false)" in imported
    assert "managed.record = journal.record" in imported
    assert "ManagedSlotState::ARCHIVED" in imported
    assert "seed_slot_rolling_code_" in imported
    assert '"resumed"' in imported
    assert "runtime_program" not in imported


def test_boot_recovery_reemits_transfer_token_and_backup():
    setup = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::setup()",
        "void SomfyIohcManager::loop()",
    )
    initial = setup.split('set_timeout("initial-backups"', 1)[1]
    assert "read_transfer_journal_" in initial
    assert "publish_backup_(journal.slot)" in initial
    assert '"transfer_prepared"' in initial
    assert '"transfer_imported"' in initial
    assert '"transfer_activated"' in initial
    assert "journal.record.state" in initial
    assert "journal.token" in initial
    assert '"recovered_after_boot"' in initial


def test_destination_recovery_honors_archived_and_active_journal_phases():
    recovery = _function(
        MANAGER_CPP,
        "bool SomfyIohcManager::recover_pending_transfer_",
        "void SomfyIohcManager::stage_slot_",
    )
    source = recovery.split("ManagedTransferRole::SOURCE", 1)[1].split(
        "// Destination import", 1
    )[0]
    assert "ManagedSlotState::ARCHIVED" in source
    assert "set_runtime_enabled(false)" in recovery

    destination = recovery.split("// Destination import", 1)[1]
    assert "managed.record = journal.record" in destination
    assert "seed_slot_rolling_code_" in destination
    assert "save_record_" in destination
    assert "apply_slot_" in destination
    assert "ManagedSlotState::ACTIVE" in destination
    assert 'active ? "active, pending finalize"' in destination


def test_composite_commands_publish_completion_after_their_final_frame():
    setup = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::create_slots()",
        "void SomfyIohcManager::setup()",
    )
    assert "set_my_sequence_complete_callback" in setup
    assert "on_my_sequence_complete_(index)" in setup
    assert "set_tilt_sequence_complete_callback" in setup
    assert "on_tilt_sequence_complete_(index)" in setup

    my_complete = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::on_my_sequence_complete_",
        "void SomfyIohcManager::on_tilt_sequence_complete_",
    )
    assert 'publish_status_("command_complete", slot, "my")' in my_complete
    assert "MY_INTER_TRANSACTION_GAP_MS = 150" in MANAGER_CPP
    assert 'set_timeout("iohc-my-next", MY_INTER_TRANSACTION_GAP_MS' in my_complete

    my_sequence = _function(
        COVER_CPP,
        "bool SomfyIohcCover::send_1w_my_sequence",
        "void SomfyIohcCover::send_2w_command",
    )
    release = my_sequence.split('set_timeout("iohc-my-release"', 1)[1]
    assert release.index("send_1w_button_event") < release.index(
        "finish_1w_my_sequence_"
    )

    tilt_complete = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::on_tilt_sequence_complete_",
        "void SomfyIohcManager::restore_service",
    )
    assert 'set_timeout(timeout_name, 1' in tilt_complete
    assert 'publish_status_("command_complete", slot, "tilt")' in tilt_complete

    tilt_sequence = _function(
        COVER_CPP,
        "void SomfyIohcCover::send_next_tilt_step_",
        "void SomfyIohcCover::cancel_1w_tilt_sequence",
    )
    assert tilt_sequence.index("send_1w_button_event") < tilt_sequence.index(
        "finish_1w_tilt_sequence_"
    )


def test_dynamic_slots_own_their_nvs_identity_strings():
    """Empty/reset slots must not retain pointers into temporary records."""

    assert "std::string storage_key_;" in COVER_H
    assert "std::string storage_namespace_;" in COVER_H
    assert "this->storage_namespace_ = ns == nullptr ? \"\" : ns;" in COVER_CPP
    assert "this->storage_key_ = key == nullptr ? \"\" : key;" in COVER_CPP
    assert "this->storage_namespace_.c_str()" in COVER_CPP
    assert "this->storage_key_.c_str()" in COVER_CPP

    # The NVS adapter also owns a copy so callers can safely pass local strings.
    assert "std::string name_;" in STORAGE_H
    assert "std::string key_;" in STORAGE_H
    assert "this->name_.c_str()" in STORAGE_CPP
    assert "this->key_.c_str()" in STORAGE_CPP


def test_rx_stats_expose_raw_frequency_diagnostics_before_crc_validation():
    on_packet = _function(
        HUB_CPP,
        "void SomfyIohcHub::on_packet",
        "void SomfyIohcHub::handle_2w_packet_",
    )
    assert "last_raw_frequency_offset_ = freq_offset" in on_packet
    assert "last_raw_rssi_ = rssi" in on_packet
    stats = _function(
        MANAGER_CPP,
        "void SomfyIohcManager::publish_rx_stats_",
        "void SomfyIohcManager::publish_backup_",
    )
    assert 'raw_rssi=%.1f,freq_offset=%.0f' in stats
    assert "get_last_raw_frequency_offset()" in stats
