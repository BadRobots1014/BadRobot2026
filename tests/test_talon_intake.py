"""Unit tests for TalonIntakeSubsystem."""

from unittest.mock import MagicMock

from phoenix6.controls.voltage_out import VoltageOut
import pytest

from subsystems.intake import (
    DUMP_VOLTAGE,
    EXTENSION_VOLTAGE,
    INTAKE_VOLTAGE,
    IntakeSubsystem,
)


def _controlled_voltage(mock: MagicMock) -> float:
    """Return the output voltage from the most recent set_control(VoltageOut(v)) call."""
    call_arg = mock.set_control.call_args[0][0]
    assert isinstance(call_arg, VoltageOut)
    return call_arg.output


@pytest.fixture
def talon_intake() -> IntakeSubsystem:
    intake_motor = MagicMock()
    left = MagicMock()
    left.device_id = 1  # Follower requires an integer device_id
    right = MagicMock()
    forward = MagicMock()
    backward = MagicMock()
    # Constructor signature: (intake, right, left, forward, backward)
    subsystem = IntakeSubsystem(intake_motor, right, left, forward, backward)
    subsystem.intake_motor.reset_mock()
    subsystem.left.reset_mock()
    return subsystem


def test_default_voltages(talon_intake: IntakeSubsystem) -> None:
    assert talon_intake.intake_voltage == INTAKE_VOLTAGE
    assert talon_intake.dump_voltage == DUMP_VOLTAGE
    assert talon_intake.extension_voltage == EXTENSION_VOLTAGE


def test_set_intake_voltage(talon_intake: IntakeSubsystem) -> None:
    talon_intake.set_intake_voltage(4.5)
    talon_intake.intake_motor.set_voltage.assert_called_once_with(4.5)


def test_set_intake_voltage_from_nt(talon_intake: IntakeSubsystem) -> None:
    talon_intake.intake_voltage = 3.5
    talon_intake.set_intake_voltage_from_networktable()
    talon_intake.intake_motor.set_voltage.assert_called_once_with(3.5)


def test_set_dump_voltage_from_nt(talon_intake: IntakeSubsystem) -> None:
    talon_intake.dump_voltage = -4.0
    talon_intake.set_dump_voltage_from_networktable()
    talon_intake.intake_motor.set_voltage.assert_called_once_with(-4.0)


def test_forward_extended_delegates_to_switch(
    talon_intake: IntakeSubsystem,
) -> None:
    talon_intake.forward.get_state.return_value = True
    assert talon_intake.forward_extended() is True


def test_backward_extended_delegates_to_switch(
    talon_intake: IntakeSubsystem,
) -> None:
    talon_intake.backward.get_state.return_value = False
    assert talon_intake.backward_extended() is False


def test_extension_voltage_blocked_at_forward_limit(
    talon_intake: IntakeSubsystem,
) -> None:
    """Positive voltage must be zeroed when forward limit is triggered."""
    talon_intake.forward.get_state.return_value = True
    talon_intake.backward.get_state.return_value = False
    talon_intake.set_extension_voltage(3.0)
    assert _controlled_voltage(talon_intake.left) == 0


def test_extension_voltage_blocked_at_backward_limit(
    talon_intake: IntakeSubsystem,
) -> None:
    """Negative voltage must be zeroed when backward limit is triggered."""
    talon_intake.forward.get_state.return_value = False
    talon_intake.backward.get_state.return_value = True
    talon_intake.set_extension_voltage(-3.0)
    assert _controlled_voltage(talon_intake.left) == 0


def test_extension_voltage_runs_with_no_limit(
    talon_intake: IntakeSubsystem,
) -> None:
    talon_intake.forward.get_state.return_value = False
    talon_intake.backward.get_state.return_value = False
    talon_intake.set_extension_voltage(3.0)
    assert _controlled_voltage(talon_intake.left) == 3.0


def test_retraction_allowed_when_only_forward_limit_hit(
    talon_intake: IntakeSubsystem,
) -> None:
    """Forward limit must not prevent retraction."""
    talon_intake.forward.get_state.return_value = True
    talon_intake.backward.get_state.return_value = False
    talon_intake.set_extension_voltage(-3.0)
    assert _controlled_voltage(talon_intake.left) == -3.0


def test_extension_allowed_when_only_backward_limit_hit(
    talon_intake: IntakeSubsystem,
) -> None:
    """Backward limit must not prevent extension."""
    talon_intake.forward.get_state.return_value = False
    talon_intake.backward.get_state.return_value = True
    talon_intake.set_extension_voltage(3.0)
    assert _controlled_voltage(talon_intake.left) == 3.0


def test_extension_from_nt_runs_when_not_at_forward_limit(
    talon_intake: IntakeSubsystem,
) -> None:
    talon_intake.extension_voltage = 3.0
    talon_intake.forward.get_state.return_value = False
    talon_intake.set_extension_voltage_from_networktable()
    assert _controlled_voltage(talon_intake.left) == 3.0


def test_extension_from_nt_stops_when_forward_limit(
    talon_intake: IntakeSubsystem,
) -> None:
    talon_intake.extension_voltage = 3.0
    talon_intake.forward.get_state.return_value = True
    talon_intake.set_extension_voltage_from_networktable()
    assert _controlled_voltage(talon_intake.left) == 0


def test_retraction_from_nt_runs_when_not_at_backward_limit(
    talon_intake: IntakeSubsystem,
) -> None:
    talon_intake.extension_voltage = 3.0
    talon_intake.backward.get_state.return_value = False
    talon_intake.set_retraction_voltage_from_networktable()
    assert _controlled_voltage(talon_intake.left) == -3.0


def test_retraction_from_nt_stops_when_backward_limit(
    talon_intake: IntakeSubsystem,
) -> None:
    talon_intake.extension_voltage = 3.0
    talon_intake.backward.get_state.return_value = True
    talon_intake.set_retraction_voltage_from_networktable()
    assert _controlled_voltage(talon_intake.left) == 0
