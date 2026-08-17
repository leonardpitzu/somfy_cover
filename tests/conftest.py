"""Mock ESPHome modules so component Python code can be imported without ESPHome installed."""

import os
import sys
from unittest.mock import MagicMock


def _install_esphome_mocks():
    """Populate sys.modules with lightweight mocks of every esphome sub-package
    that components/somfy/ imports at the top level."""

    esphome = MagicMock()

    # --- esphome.config_validation ---
    cv = MagicMock()

    class Invalid(Exception):
        """Stand-in for voluptuous / esphome config validation errors."""

    cv.Invalid = Invalid
    cv.string = lambda value: str(value)
    esphome.config_validation = cv

    # --- esphome.final_validate ---
    final_validate = MagicMock()
    esphome.final_validate = final_validate

    # --- esphome.const (only the symbols our modules actually import) ---
    const = MagicMock()
    const.CONF_CLOSE_DURATION = "close_duration"
    const.CONF_ID = "id"
    const.CONF_OPEN_DURATION = "open_duration"
    const.PLATFORM_ESP32 = "esp32"
    const.CONF_TYPE = "type"
    esphome.const = const

    # --- sub-packages imported by __init__.py and cover.py ---
    components = MagicMock()
    esphome.components = components

    modules = {
        "esphome": esphome,
        "esphome.codegen": esphome.codegen,
        "esphome.config_validation": cv,
        "esphome.final_validate": final_validate,
        "esphome.const": const,
        "esphome.components": components,
        "esphome.components.button": components.button,
        "esphome.components.remote_transmitter": components.remote_transmitter,
        "esphome.components.remote_receiver": components.remote_receiver,
        "esphome.components.text_sensor": components.text_sensor,
        "esphome.components.cover": components.cover,
    }

    sys.modules.update(modules)


# Install mocks before any test module tries to import from the component.
_install_esphome_mocks()

# Add the components directory to sys.path so `from somfy.cover import …` works.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "components"))
