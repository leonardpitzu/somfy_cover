"""Regression checks for hardware-verified IOHC Venetian tilt support."""

from pathlib import Path

ROOT = Path(__file__).parent.parent
COVER_CPP = (ROOT / "components/somfy/somfy_iohc.cpp").read_text()
COVER_H = (ROOT / "components/somfy/somfy_iohc.h").read_text()
MANAGER_CPP = (
    ROOT / "components/somfy_iohc_manager/somfy_iohc_manager.cpp"
).read_text()
MANAGER_H = (
    ROOT / "components/somfy_iohc_manager/somfy_iohc_manager.h"
).read_text()
HUB_CPP = (ROOT / "components/somfy/somfy_hub_iohc.cpp").read_text()
HUB_H = (ROOT / "components/somfy/somfy_hub_iohc.h").read_text()
SOMFY_PY = (ROOT / "components/somfy/__init__.py").read_text()


def test_captured_wheel_vectors_are_encoded_exactly():
    assert "BUTTON_ACTION_TILT_CLOCKWISE = 0x0D" in COVER_H
    assert "BUTTON_ACTION_TILT_COUNTERCLOCKWISE = 0x0E" in COVER_H
    assert "clockwise ? 0xCD : 0xCC" in COVER_CPP
    assert "clockwise ? 0x2E : 0xA2" in COVER_CPP
    assert "send_1w_button_event(action, false)" in COVER_CPP


def test_situo_my_uses_extended_execute_but_stop_stays_distinct():
    assert "bool SomfyIohcCover::send_1w_my_execute()" in COVER_CPP
    assert "0xD2," in COVER_CPP
    assert 'set_timeout("iohc-my-recall"' in COVER_CPP
    assert "this->send_1w_my_execute()" in COVER_CPP
    assert "this->send_1w_command(iohc_cmd::MP_STOP)" in COVER_CPP


def test_tilt_percentages_use_native_cover_traits_and_endpoint_sync():
    assert "traits.set_supports_tilt(this->venetian_)" in COVER_CPP
    assert "call.get_tilt()" in COVER_CPP
    assert "const bool endpoint" in COVER_CPP
    assert "TILT_ENDPOINT_MARGIN_STEPS" in COVER_CPP
    assert "target_step / static_cast<float>(this->tilt_steps_)" in COVER_CPP
    assert "reachable_target" in COVER_CPP
    assert "runtime_set_tilt" in COVER_H


def test_manager_keeps_record_v1_layout_and_preserves_features_on_moves():
    assert "uint16_t flags;" in MANAGER_H
    assert "constexpr uint8_t RECORD_VERSION = 1" in MANAGER_CPP
    assert "FLAG_VENETIAN = 0x4000" in MANAGER_CPP
    assert "original.flags & FLAG_FEATURE_MASK" in MANAGER_CPP
    assert "target.record.flags &= FLAG_FEATURE_MASK" in MANAGER_CPP
    assert "DURATION_MS_MASK = 0x00FFFFFFUL" in MANAGER_CPP
    assert "set_record_my_tilt_step" in MANAGER_CPP
    assert "record_close_duration_ms(record)" in MANAGER_CPP


def test_manager_exposes_backward_compatible_separate_venetian_service():
    assert '"somfy_calibrate"' in MANAGER_CPP
    assert '"somfy_venetian"' in MANAGER_CPP
    assert 'command == "tilt_position"' in MANAGER_CPP
    assert 'command == "tilt_stop"' in MANAGER_CPP
    assert '"my_tilt_step"' in MANAGER_CPP


def test_lift_and_my_commands_publish_verified_venetian_tilt_states():
    assert "this->set_lift_tilt_(true)" in COVER_CPP
    assert "this->set_lift_tilt_(false)" in COVER_CPP
    assert "this->my_tilt_pending_ = this->venetian_" in COVER_CPP
    assert "this->set_my_tilt_(false)" in COVER_CPP
    assert "this->stop_rx_sync();" in COVER_CPP
    lift_tilt = COVER_CPP.split(
        "void SomfyIohcCover::set_lift_tilt_", 1
    )[1].split("void SomfyIohcCover::set_my_tilt_", 1)[0]
    assert "this->tilt = opening ? 1.0f : 0.0f;" in lift_tilt
    assert "? 0.5f" not in lift_tilt


def test_remote_discovery_window_allows_gui_interaction_time():
    assert "DISCOVERY_TIMEOUT_MS = 120000" in MANAGER_CPP


def test_calibrated_1w_frequency_survives_every_transmit_cycle():
    assert 'CONF_FREQUENCY_1W = "frequency_1w"' in SOMFY_PY
    configure = HUB_CPP.split("void SomfyIohcHub::configure_radio_1w()", 1)[1]
    configure = configure.split("void SomfyIohcHub::configure_radio_2w", 1)[0]
    assert "this->frequency_1w_" in configure
    assert "iohc::FREQUENCY_1W" not in configure


def test_1w_rx_uses_hardware_verified_full_capture_window():
    assert "RX_FIFO_WINDOW_1W = 60" in HUB_H
    assert "RX_FIFO_WINDOW_2W = 60" in HUB_H
    one_way = HUB_CPP.split("void SomfyIohcHub::configure_radio_1w()", 1)[1]
    one_way = one_way.split("void SomfyIohcHub::configure_radio_2w", 1)[0]
    assert "RX_FIFO_WINDOW_1W" in one_way
    two_way = HUB_CPP.split("void SomfyIohcHub::configure_radio_2w", 1)[1]
    two_way = two_way.split("void SomfyIohcHub::start_2w_listen", 1)[0]
    assert "RX_FIFO_WINDOW_2W" in two_way


def test_1w_rx_uses_relative_carrier_detection_for_weak_remotes():
    one_way = HUB_CPP.split("void SomfyIohcHub::configure_radio_1w()", 1)[1]
    one_way = one_way.split("void SomfyIohcHub::configure_radio_2w", 1)[0]
    assert "set_magn_target(cc1101::MagnTarget::MAGN_TARGET_33DB)" in one_way
    assert "set_max_lna_gain(cc1101::MaxLnaGain::MAX_LNA_GAIN_DEFAULT)" in one_way
    assert "set_max_dvga_gain(cc1101::MaxDvgaGain::MAX_DVGA_GAIN_DEFAULT)" in one_way
    assert "set_lna_priority(true)" in one_way
    assert "set_carrier_sense_abs_thr(-8)" in one_way
    assert "CARRIER_SENSE_REL_THR_PLUS_6DB" in one_way
    assert "set_sync_mode(cc1101::SyncMode::SYNC_MODE_16_16)" in one_way
    assert "set_carrier_sense_above_threshold(true)" in one_way


def test_1w_tx_restores_receive_specific_radio_settings():
    transmit = HUB_CPP.split("void SomfyIohcHub::transmit_packet", 1)[1]
    transmit = transmit.split("void SomfyIohcHub::begin_rx", 1)[0]
    assert "this->configure_radio_1w();\n  this->cc1101_->begin_rx();" in transmit


def test_manager_exposes_receive_pipeline_counters_on_demand():
    assert "rx_raw_packet_count_" in HUB_H
    assert "rx_valid_frame_count_" in HUB_H
    assert "accepted_remote_command_count_" in MANAGER_H
    assert 'action == "rx_stats"' in MANAGER_CPP
    assert 'this->publish_status_("rx_stats"' in MANAGER_CPP
    assert '"raw=%" PRIu32 ",valid=%" PRIu32 ",accepted=%" PRIu32' in MANAGER_CPP
