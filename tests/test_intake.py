"""Unit tests for TalonIntakeSubsystem."""

from unittest.mock import MagicMock

from phoenix6.controls.voltage_out import VoltageOut
import pytest

from subsystems.intake import (
    DUMP_VOLTAGE,
    INTAKE_VOLTAGE,
    IntakeSubsystem,
)


def _controlled_voltage(mock: MagicMock) -> float:
    """Return the output voltage from the most recent set_control(VoltageOut(v)) call."""
    call_arg = mock.set_control.call_args[0][0]
    assert isinstance(call_arg, VoltageOut)
    return call_arg.output


@pytest.fixture
def intake() -> IntakeSubsystem:
    intake_motor = MagicMock()
    subsystem = IntakeSubsystem(intake_motor)
    subsystem.intake_motor.reset_mock()
    return subsystem


def test_default_voltages(intake: IntakeSubsystem) -> None:
    assert intake.intake_voltage == INTAKE_VOLTAGE
    assert intake.dump_voltage == DUMP_VOLTAGE


def test_set_intake_voltage(intake: IntakeSubsystem) -> None:
    intake.set_intake_voltage(4.5)
    intake.intake_motor.set_voltage.assert_called_once_with(4.5)


def test_set_intake_voltage_from_nt(intake: IntakeSubsystem) -> None:
    intake.intake_voltage = 3.5
    intake.set_intake_voltage_from_networktable()
    intake.intake_motor.set_voltage.assert_called_once_with(3.5)


def test_set_dump_voltage_from_nt(intake: IntakeSubsystem) -> None:
    intake.dump_voltage = -4.0
    intake.set_dump_voltage_from_networktable()
    intake.intake_motor.set_voltage.assert_called_once_with(-4.0)
