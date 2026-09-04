"""Guardrails for the receive-only IOHC discovery component."""

import importlib.util
from pathlib import Path


ROOT = Path(__file__).parent.parent


def _load_capture_module():
    path = ROOT / "components" / "somfy_iohc_capture" / "__init__.py"
    spec = importlib.util.spec_from_file_location("somfy_iohc_capture", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


capture = _load_capture_module()


def test_remote_filter_accepts_only_valid_iohc_node_ids():
    assert capture.cv.hex_int_range(min=1, max=0xFFFFFF) is not None


def test_decoded_packet_exposes_complete_callback_lifetime_frame():
    header = (ROOT / "components" / "somfy" / "somfy_hub_iohc.h").read_text()
    source = (ROOT / "components" / "somfy" / "somfy_hub_iohc.cpp").read_text()
    assert "uint8_t ctrl0" in header
    assert "uint8_t ctrl1" in header
    assert "const uint8_t *frame" in header
    assert "size_t frame_len" in header
    assert "pkt.frame = packet.data()" in source
    assert "pkt.frame_len = packet.size()" in source


def test_capture_component_cannot_transmit_or_pair():
    directory = ROOT / "components" / "somfy_iohc_capture"
    implementation = "\n".join(
        path.read_text() for path in directory.iterdir() if path.suffix in {".h", ".cpp"}
    )
    forbidden = (
        "transmit_packet(",
        "send_2w_command(",
        "runtime_program(",
        "send_1w_command(",
        "CustomAPIDevice",
    )
    for token in forbidden:
        assert token not in implementation


def test_capture_keeps_every_rf_copy_observable():
    source = (
        ROOT
        / "components"
        / "somfy_iohc_capture"
        / "somfy_iohc_capture.cpp"
    ).read_text()
    assert "++this->event_counter_" in source
    assert "publish_state(state)" in source
    assert "is_duplicate" not in source
