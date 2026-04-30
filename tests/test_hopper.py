"""Unit tests for HopperSubsystem."""

from unittest.mock import MagicMock

from phoenix6.controls.voltage_out import VoltageOut
import pytest

from subsystems.hopper import (
    EXTENSION_VOLTAGE,
    HopperSubsystem,
)


def _controlled_voltage(mock: MagicMock) -> float:
    """Return the output voltage from the most recent set_control(VoltageOut(v)) call."""
    call_arg = mock.set_control.call_args[0][0]
    assert isinstance(call_arg, VoltageOut)
    return call_arg.output


@pytest.fixture
def hopper() -> HopperSubsystem:
    left_motor = MagicMock()
    left_motor.device_id = 1  # Follower requires an integer device_id
    right_motor = MagicMock()
    forward_limit_switch = MagicMock()
    subsystem = HopperSubsystem(right_motor, left_motor, forward_limit_switch)
    subsystem.left_motor.reset_mock()
    return subsystem


def test_default_voltages(hopper: HopperSubsystem) -> None:
    assert hopper.extension_voltage == EXTENSION_VOLTAGE


def test_forward_extended_delegates_to_switch(
    hopper: HopperSubsystem,
) -> None:
    hopper.forward_limit_switch.get_state.return_value = True
    assert hopper.is_forward_extended() is True


def test_extension_voltage_blocked_at_forward_limit(
    hopper: HopperSubsystem,
) -> None:
    """Positive voltage must be zeroed when forward limit is triggered."""
    hopper.forward_limit_switch.get_state.return_value = True
    hopper.set_extension_voltage(3.0)
    assert _controlled_voltage(hopper.left_motor) == 0


def test_extension_voltage_runs_with_no_limit(
    hopper: HopperSubsystem,
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    hopper.set_extension_voltage(3.0)
    assert _controlled_voltage(hopper.left_motor) == 3.0


def test_retraction_not_allowed_when_only_forward_limit_hit(
    hopper: HopperSubsystem,
) -> None:
    """Forward limit must not prevent retraction."""
    hopper.forward_limit_switch.get_state.return_value = True
    hopper.set_extension_voltage(-3.0)
    assert _controlled_voltage(hopper.left_motor) == 0


def test_extension_from_nt_runs_when_not_at_forward_limit(
    hopper: HopperSubsystem,
) -> None:
    hopper.extension_voltage = 3.0
    hopper.forward_limit_switch.get_state.return_value = False
    hopper.set_extension_voltage_from_networktable()
    assert _controlled_voltage(hopper.left_motor) == -3.0


def test_extension_from_nt_stops_when_forward_limit(
    hopper: HopperSubsystem,
) -> None:
    hopper.extension_voltage = 3.0
    hopper.forward_limit_switch.get_state.return_value = True
    hopper.set_extension_voltage_from_networktable()
    assert _controlled_voltage(hopper.left_motor) == 0
