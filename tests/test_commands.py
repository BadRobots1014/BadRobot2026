"""Unit tests for robot commands."""
from unittest.mock import MagicMock, patch

import pytest

from commands.bang_bang_shoot import BangBangShootCommand
from commands.climb import ClimbCommand
from commands.extend_hopper import ExtendHopperCommand
from commands.shoot import SHOOT_VELOCITY, ShootCommand
from commands.shoot_kicker import KICKER_VOLTAGE, ShootKickerCommand
from subsystems.climber import ClimberSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem


@pytest.fixture
def shooter() -> ShooterSubsystem:
    return ShooterSubsystem(
        MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()
    )


@pytest.fixture
def intake() -> IntakeSubsystem:
    return IntakeSubsystem(
        MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()
    )


@pytest.fixture
def climber() -> ClimberSubsystem:
    return ClimberSubsystem(MagicMock())


# --- BangBangShootCommand ---


def test_bang_bang_applies_full_voltage_when_below_target(
    shooter: ShooterSubsystem,
) -> None:
    shooter.shoot_velocity = 4500
    shooter.shoot_encoder.get_velocity.return_value = 3000.0
    BangBangShootCommand(shooter).execute()
    shooter.shoot_motor.set_voltage.assert_called_once_with(12)


def test_bang_bang_applies_zero_when_at_or_above_target(
    shooter: ShooterSubsystem,
) -> None:
    shooter.shoot_velocity = 4500
    shooter.shoot_encoder.get_velocity.return_value = 5000.0
    BangBangShootCommand(shooter).execute()
    shooter.shoot_motor.set_voltage.assert_called_once_with(0)


# --- ShootCommand ---


def test_shoot_command_sets_configured_velocity(shooter: ShooterSubsystem) -> None:
    with patch("robot.TEST_MODE_ENABLED", False):
        ShootCommand(shooter).execute()
    shooter.shoot_motor.set_velocity.assert_called_once_with(SHOOT_VELOCITY)


# --- ShootKickerCommand ---


def test_shoot_kicker_inverted_applies_negative_voltage(
    shooter: ShooterSubsystem,
) -> None:
    ShootKickerCommand(shooter, invert=True).execute()
    shooter.kick_motor.set_voltage.assert_called_once_with(-KICKER_VOLTAGE)


def test_shoot_kicker_normal_applies_positive_voltage(
    shooter: ShooterSubsystem,
) -> None:
    with patch("robot.TEST_MODE_ENABLED", False):
        ShootKickerCommand(shooter, invert=False).execute()
    shooter.kick_motor.set_voltage.assert_called_once_with(KICKER_VOLTAGE)


# --- ExtendHopperCommand ---


def test_extend_finished_when_forward_limit_hit(intake: IntakeSubsystem) -> None:
    intake.forward.get_state.return_value = True
    assert ExtendHopperCommand(intake, extend=True).isFinished() is True


def test_extend_not_finished_without_forward_limit(intake: IntakeSubsystem) -> None:
    intake.forward.get_state.return_value = False
    assert ExtendHopperCommand(intake, extend=True).isFinished() is False


def test_retract_finished_when_backward_limit_hit(intake: IntakeSubsystem) -> None:
    intake.backward.get_state.return_value = True
    assert ExtendHopperCommand(intake, extend=False).isFinished() is True


def test_retract_not_finished_without_backward_limit(intake: IntakeSubsystem) -> None:
    intake.backward.get_state.return_value = False
    assert ExtendHopperCommand(intake, extend=False).isFinished() is False


# --- ClimbCommand ---


def test_climb_extend_applies_positive_voltage(climber: ClimberSubsystem) -> None:
    ClimbCommand(climber, extend=True).execute()
    voltage = climber.climb_motor.set_voltage.call_args[0][0]
    assert voltage > 0


def test_climb_retract_applies_negative_voltage(climber: ClimberSubsystem) -> None:
    ClimbCommand(climber, extend=False).execute()
    voltage = climber.climb_motor.set_voltage.call_args[0][0]
    assert voltage < 0


def test_climb_end_stops_motor(climber: ClimberSubsystem) -> None:
    ClimbCommand(climber).end(interrupted=False)
    climber.climb_motor.set_voltage.assert_called_once_with(0)