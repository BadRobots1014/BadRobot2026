"""Unit tests for IntakeSubsystem."""

from unittest.mock import MagicMock

import pytest

from subsystems.intake import (
    DUMP_VOLTAGE,
    EXTENSION_VOLTAGE,
    INTAKE_VOLTAGE,
    IntakeSubsystem,
)


@pytest.fixture
def intake() -> IntakeSubsystem:
    subsystem = IntakeSubsystem(
        MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()
    )
    # Reset calls from __init__ so assertions only cover the method under test.
    subsystem.intake_motor.reset_mock()
    subsystem.left.reset_mock()
    return subsystem


def test_default_voltages(intake: IntakeSubsystem) -> None:
    assert intake.intake_voltage == INTAKE_VOLTAGE
    assert intake.dump_voltage == DUMP_VOLTAGE
    assert intake.extension_voltage == EXTENSION_VOLTAGE


def test_set_intake_voltage(intake: IntakeSubsystem) -> None:
    intake.set_intake_voltage(4.5)
    intake.intake_motor.set_voltage.assert_called_once_with(4.5)


def test_set_intake_velocity(intake: IntakeSubsystem) -> None:
    intake.set_intake_velocity(1500.0)
    intake.intake_motor.set_velocity.assert_called_once_with(1500.0)


def test_set_intake_voltage_from_nt(intake: IntakeSubsystem) -> None:
    intake.intake_voltage = 3.5
    intake.set_intake_voltage_from_networktable()
    intake.intake_motor.set_voltage.assert_called_once_with(3.5)


def test_set_dump_voltage_from_nt(intake: IntakeSubsystem) -> None:
    intake.dump_voltage = -4.0
    intake.set_dump_voltage_from_networktable()
    intake.intake_motor.set_voltage.assert_called_once_with(-4.0)


def test_forward_extended_delegates_to_switch(intake: IntakeSubsystem) -> None:
    intake.forward.get_state.return_value = True
    assert intake.forward_extended() is True


def test_backward_extended_delegates_to_switch(intake: IntakeSubsystem) -> None:
    intake.backward.get_state.return_value = False
    assert intake.backward_extended() is False


def test_extension_voltage_blocked_at_forward_limit(intake: IntakeSubsystem) -> None:
    """Positive voltage must be zeroed when the forward limit switch is triggered."""
    intake.forward.get_state.return_value = True
    intake.backward.get_state.return_value = False
    intake.set_extension_voltage(3.0)
    intake.left.set_voltage.assert_called_once_with(0)


def test_extension_voltage_blocked_at_backward_limit(intake: IntakeSubsystem) -> None:
    """Negative voltage must be zeroed when the backward limit switch is triggered."""
    intake.forward.get_state.return_value = False
    intake.backward.get_state.return_value = True
    intake.set_extension_voltage(-3.0)
    intake.left.set_voltage.assert_called_once_with(0)


def test_extension_voltage_runs_with_no_limit(intake: IntakeSubsystem) -> None:
    intake.forward.get_state.return_value = False
    intake.backward.get_state.return_value = False
    intake.set_extension_voltage(3.0)
    intake.left.set_voltage.assert_called_once_with(3.0)


def test_retraction_allowed_when_only_forward_limit_hit(
    intake: IntakeSubsystem,
) -> None:
    """Forward limit must not prevent retraction."""
    intake.forward.get_state.return_value = True
    intake.backward.get_state.return_value = False
    intake.set_extension_voltage(-3.0)
    intake.left.set_voltage.assert_called_once_with(-3.0)


def test_extension_allowed_when_only_backward_limit_hit(
    intake: IntakeSubsystem,
) -> None:
    """Backward limit must not prevent extension."""
    intake.forward.get_state.return_value = False
    intake.backward.get_state.return_value = True
    intake.set_extension_voltage(3.0)
    intake.left.set_voltage.assert_called_once_with(3.0)


def test_extension_from_nt_runs_when_not_extended(intake: IntakeSubsystem) -> None:
    intake.extension_voltage = 3.0
    intake.forward.get_state.return_value = False
    intake.set_extension_voltage_from_networktable()
    intake.left.set_voltage.assert_called_once_with(3.0)


def test_extension_from_nt_stops_when_forward_limit(intake: IntakeSubsystem) -> None:
    intake.extension_voltage = 3.0
    intake.forward.get_state.return_value = True
    intake.set_extension_voltage_from_networktable()
    intake.left.set_voltage.assert_called_once_with(0)


def test_retraction_from_nt_runs_when_not_retracted(intake: IntakeSubsystem) -> None:
    intake.extension_voltage = 3.0
    intake.backward.get_state.return_value = False
    intake.set_retraction_voltage_from_networktable()
    intake.left.set_voltage.assert_called_once_with(-3.0)


def test_retraction_from_nt_stops_when_backward_limit(intake: IntakeSubsystem) -> None:
    intake.extension_voltage = 3.0
    intake.backward.get_state.return_value = True
    intake.set_retraction_voltage_from_networktable()
    intake.left.set_voltage.assert_called_once_with(0)
