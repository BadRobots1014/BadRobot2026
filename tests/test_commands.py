"""Unit tests for robot commands."""

from unittest.mock import MagicMock, patch

import pytest

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import DUMP_VOLTAGE, INTAKE_VOLTAGE, RunIntakeCommand
from commands.run_kicker import KICKER_VOLTAGE, RunKickerCommand
from commands.run_shooter import RunShooterCommand
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem


@pytest.fixture
def shooter() -> ShooterSubsystem:
    return ShooterSubsystem(MagicMock(), MagicMock(), MagicMock())


@pytest.fixture
def kicker() -> KickerSubsystem:
    return KickerSubsystem(MagicMock(), MagicMock())


@pytest.fixture
def intake() -> IntakeSubsystem:
    return IntakeSubsystem(MagicMock())


@pytest.fixture
def hopper() -> HopperSubsystem:
    return HopperSubsystem(MagicMock(), MagicMock(), MagicMock(), MagicMock())


# --- ShootCommand ---


def test_shoot_command_sets_configured_velocity(shooter: ShooterSubsystem) -> None:
    RunShooterCommand(shooter, 4000).execute()
    shooter.shoot_motor.set_velocity.assert_called_once_with(4000)


# --- ShootKickerCommand ---


def test_shoot_kicker_inverted_applies_negative_voltage(
    kicker: KickerSubsystem,
) -> None:
    RunKickerCommand(kicker, invert=True).execute()
    kicker.kick_motor.set_voltage.assert_called_once_with(-KICKER_VOLTAGE)


def test_shoot_kicker_normal_applies_positive_voltage(
    kicker: KickerSubsystem,
) -> None:
    with patch("robot.TEST_MODE_ENABLED", new=False):
        RunKickerCommand(kicker, invert=False).execute()
    kicker.kick_motor.set_voltage.assert_called_once_with(KICKER_VOLTAGE)


# --- ExtendHopperCommand ---


def test_extend_finished_when_forward_limit_hit(
    hopper: HopperSubsystem
) -> None:
    hopper.forward_limit_switch.get_state.return_value = True
    assert ExtendHopperCommand(hopper, extend=True).isFinished() is True


def test_extend_not_finished_without_forward_limit(
    hopper: HopperSubsystem
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    assert ExtendHopperCommand(hopper, extend=True).isFinished() is False


def test_retract_finished_when_backward_limit_hit(
    hopper: HopperSubsystem,
) -> None:
    hopper.backward_limit_switch.get_state.return_value = True
    assert ExtendHopperCommand(hopper, extend=False).isFinished() is True


def test_retract_not_finished_without_backward_limit(
    hopper: HopperSubsystem,
) -> None:
    hopper.backward_limit_switch.get_state.return_value = False
    assert ExtendHopperCommand(hopper, extend=False).isFinished() is False


# --- ShootKickerCommand (test mode) ---


def test_shoot_kicker_uses_nt_voltage_when_test_mode_and_not_inverted(
    kicker: KickerSubsystem,
) -> None:
    kicker.kick_shoot_voltage = 3.5
    with patch("robot.TEST_MODE_ENABLED", new=True):
        RunKickerCommand(kicker, invert=False).execute()
    kicker.kick_motor.set_voltage.assert_called_once_with(3)


# --- ExtendHopperCommand execute() ---


def test_extend_hopper_execute_extends_with_positive_voltage(
    hopper: HopperSubsystem,
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    hopper.backward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=False):
        ExtendHopperCommand(hopper, extend=True).execute()
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output > 0


def test_extend_hopper_execute_retracts_with_negative_voltage(
    hopper: HopperSubsystem,
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    hopper.backward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=False):
        ExtendHopperCommand(hopper, extend=False).execute()
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output < 0


def test_extend_hopper_execute_uses_nt_when_test_mode_extend(
    hopper: HopperSubsystem
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=True):
        ExtendHopperCommand(hopper, extend=True).execute()
    # set_extension_voltage_from_networktable calls left.set_voltage(extension_voltage)
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output == hopper.extension_voltage


def test_extend_hopper_execute_uses_nt_when_test_mode_retract(
    hopper: HopperSubsystem,
) -> None:
    hopper.backward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=True):
        ExtendHopperCommand(hopper, extend=False).execute()
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output == -hopper.extension_voltage


# --- ExtendHopperCommand end() ---


def test_extend_hopper_end_stops_motor(
    hopper: HopperSubsystem,
) -> None:
    ExtendHopperCommand(hopper, extend=True).end(interrupted=False)
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output == 0


# --- RunIntakeCommand ---


def test_run_intake_applies_intake_voltage_in_normal_mode(
    intake: IntakeSubsystem,
) -> None:
    with patch("robot.TEST_MODE_ENABLED", new=False):
        RunIntakeCommand(intake, dump=False).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(INTAKE_VOLTAGE)


def test_run_intake_uses_nt_voltage_in_test_mode(intake: IntakeSubsystem) -> None:
    intake.intake_voltage = 3.0
    with patch("robot.TEST_MODE_ENABLED", new=True):
        RunIntakeCommand(intake, dump=False).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(3.0)


def test_run_dump_applies_dump_voltage_in_normal_mode(
    intake: IntakeSubsystem,
) -> None:
    with patch("robot.TEST_MODE_ENABLED", new=False):
        RunIntakeCommand(intake, dump=True).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(DUMP_VOLTAGE)


def test_run_dump_uses_nt_voltage_in_test_mode(intake: IntakeSubsystem) -> None:
    intake.dump_voltage = -3.5
    with patch("robot.TEST_MODE_ENABLED", new=True):
        RunIntakeCommand(intake, dump=True).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(-3.5)


def test_run_intake_end_stops_motor(intake: IntakeSubsystem) -> None:
    RunIntakeCommand(intake, dump=False).end(interrupted=False)
    intake.intake_motor.set_voltage.assert_called_once_with(0)
