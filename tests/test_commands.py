"""Unit tests for robot commands."""

from unittest.mock import MagicMock, patch

import pytest

from commands.bang_bang_shoot import BangBangShootCommand
from commands.climb import ClimbCommand
from commands.extend_hopper import ExtendHopperCommand
from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from commands.run_intake import DUMP_VOLTAGE, HALF_OUT, INTAKE_VOLTAGE, RunIntakeCommand
from commands.shoot_kicker import KICKER_VOLTAGE, ShootKickerCommand
from commands.spin_shooter import SHOOT_VELOCITY, SpinShooterCommand
from subsystems.climber import ClimberSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.pilights import PiLights
from subsystems.shooter import SHOOTER_VELOCITY, ShooterSubsystem
from subsystems.talonFXIntake import TalonIntakeSubsystem


@pytest.fixture
def shooter() -> ShooterSubsystem:
    return ShooterSubsystem(MagicMock(), MagicMock(), MagicMock())


@pytest.fixture
def kicker() -> KickerSubsystem:
    return KickerSubsystem(MagicMock(), MagicMock())


@pytest.fixture
def intake() -> TalonIntakeSubsystem:
    return TalonIntakeSubsystem(
        MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock()
    )


@pytest.fixture
def climber() -> ClimberSubsystem:
    return ClimberSubsystem(MagicMock())


@pytest.fixture
def lights() -> PiLights:
    return MagicMock()


# --- BangBangShootCommand ---


def test_bang_bang_applies_full_voltage_when_below_target(
    shooter: ShooterSubsystem,
) -> None:
    shooter.shoot_velocity = 4500
    shooter.shoot_encoder.get_velocity.return_value = 3000.0
    BangBangShootCommand(shooter, None).execute()
    shooter.shoot_motor.set_voltage.assert_called_once_with(12)


def test_bang_bang_applies_zero_when_at_or_above_target(
    shooter: ShooterSubsystem,
) -> None:
    shooter.shoot_velocity = 4500
    shooter.shoot_encoder.get_velocity.return_value = 5000.0
    BangBangShootCommand(shooter, None).execute()
    shooter.shoot_motor.set_voltage.assert_called_once_with(0)


# --- ShootCommand ---


def test_shoot_command_sets_configured_velocity(shooter: ShooterSubsystem) -> None:
    SpinShooterCommand(shooter, SHOOT_VELOCITY).execute()
    shooter.shoot_motor.set_velocity.assert_called_once_with(SHOOT_VELOCITY)


# --- ShootKickerCommand ---


def test_shoot_kicker_inverted_applies_negative_voltage(
    kicker: KickerSubsystem,
) -> None:
    ShootKickerCommand(kicker, invert=True).execute()
    kicker.kick_motor.set_voltage.assert_called_once_with(-KICKER_VOLTAGE)


def test_shoot_kicker_normal_applies_positive_voltage(
    kicker: KickerSubsystem,
) -> None:
    with patch("robot.TEST_MODE_ENABLED", new=False):
        ShootKickerCommand(kicker, invert=False).execute()
    kicker.kick_motor.set_voltage.assert_called_once_with(KICKER_VOLTAGE)


# --- ExtendHopperCommand ---


def test_extend_finished_when_forward_limit_hit(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.forward.get_state.return_value = True
    assert ExtendHopperCommand(intake, lights, extend=True).isFinished() is True


def test_extend_not_finished_without_forward_limit(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.forward.get_state.return_value = False
    assert ExtendHopperCommand(intake, lights, extend=True).isFinished() is False


def test_retract_finished_when_backward_limit_hit(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.backward.get_state.return_value = True
    assert ExtendHopperCommand(intake, lights, extend=False).isFinished() is True


def test_retract_not_finished_without_backward_limit(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.backward.get_state.return_value = False
    assert ExtendHopperCommand(intake, lights, extend=False).isFinished() is False


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


# --- ShootKickerCommand (test mode) ---


def test_shoot_kicker_uses_nt_voltage_when_test_mode_and_not_inverted(
    kicker: KickerSubsystem,
) -> None:
    kicker.kick_shoot_voltage = 3.5
    with patch("robot.TEST_MODE_ENABLED", new=True):
        ShootKickerCommand(kicker, invert=False).execute()
    kicker.kick_motor.set_voltage.assert_called_once_with(3.5)


# --- ExtendHopperCommand execute() ---


def test_extend_hopper_execute_extends_with_positive_voltage(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.forward.get_state.return_value = False
    intake.backward.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=False):
        ExtendHopperCommand(intake, lights, extend=True, positive_voltage=4).execute()
    control = intake.left.set_control.call_args[0][0]
    assert control.output == 4


def test_extend_hopper_execute_retracts_with_negative_voltage(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.forward.get_state.return_value = False
    intake.backward.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=False):
        ExtendHopperCommand(intake, lights, extend=False, positive_voltage=4).execute()
    control = intake.left.set_control.call_args[0][0]
    assert control.output == -4


def test_extend_hopper_execute_uses_nt_when_test_mode_extend(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.forward.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=True):
        ExtendHopperCommand(intake, lights, extend=True).execute()
    # set_extension_voltage_from_networktable calls left.set_voltage(extension_voltage)
    control = intake.left.set_control.call_args[0][0]
    assert control.output == intake.extension_voltage


def test_extend_hopper_execute_uses_nt_when_test_mode_retract(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    intake.backward.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=True):
        ExtendHopperCommand(intake, lights, extend=False).execute()
    control = intake.left.set_control.call_args[0][0]
    assert control.output == -intake.extension_voltage


# --- ExtendHopperCommand end() ---


def test_extend_hopper_end_stops_motor(
    intake: TalonIntakeSubsystem, lights: PiLights
) -> None:
    ExtendHopperCommand(intake, lights, extend=True).end(interrupted=False)
    control = intake.left.set_control.call_args[0][0]
    assert control.output == 0


# --- RunIntakeCommand ---


def test_run_intake_applies_intake_voltage_in_normal_mode(
    intake: TalonIntakeSubsystem,
) -> None:
    with patch("robot.TEST_MODE_ENABLED", new=False):
        RunIntakeCommand(intake, dump=False).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(INTAKE_VOLTAGE)


def test_run_intake_uses_nt_voltage_in_test_mode(intake: TalonIntakeSubsystem) -> None:
    intake.intake_voltage = 3.0
    with patch("robot.TEST_MODE_ENABLED", new=True):
        RunIntakeCommand(intake, dump=False).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(3.0)


def test_run_dump_applies_dump_voltage_in_normal_mode(
    intake: TalonIntakeSubsystem,
) -> None:
    with patch("robot.TEST_MODE_ENABLED", new=False):
        RunIntakeCommand(intake, dump=True).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(DUMP_VOLTAGE)


def test_run_dump_uses_nt_voltage_in_test_mode(intake: TalonIntakeSubsystem) -> None:
    intake.dump_voltage = -3.5
    with patch("robot.TEST_MODE_ENABLED", new=True):
        RunIntakeCommand(intake, dump=True).execute()
    intake.intake_motor.set_voltage.assert_called_once_with(-3.5)


def test_run_intake_runns_over_half_out(intake: TalonIntakeSubsystem) -> None:
    intake.intake_motor.get_encoder_position.return_value = HALF_OUT + 1
    assert RunIntakeCommand(intake, dump=False).isFinished() is False


def test_run_intake_stops_half_out(intake: TalonIntakeSubsystem) -> None:
    intake.intake_motor.get_encoder_position.return_value = HALF_OUT
    assert RunIntakeCommand(intake, dump=False).isFinished() is True


def test_run_intake_end_stops_motor(intake: TalonIntakeSubsystem) -> None:
    RunIntakeCommand(intake, dump=False).end(interrupted=False)
    intake.intake_motor.set_voltage.assert_called_once_with(0)


# --- KickerShootWhenReadyCommand ---


def test_kicker_always_spins_shooter(
    shooter: ShooterSubsystem, kicker: KickerSubsystem, lights: PiLights
) -> None:
    shooter.shoot_encoder.get_velocity.return_value = 1000.0
    KickerShootWhenReadyCommand(shooter, kicker, lights, SHOOTER_VELOCITY).execute()
    shooter.shoot_motor.set_velocity.assert_called_once_with(SHOOTER_VELOCITY)


def test_kicker_fires_when_above_target_velocity(
    shooter: ShooterSubsystem, kicker: KickerSubsystem, lights: PiLights
) -> None:
    kicker.kick_shoot_voltage = 4.0
    shooter.shoot_encoder.get_velocity.return_value = 4600.0
    KickerShootWhenReadyCommand(shooter, kicker, lights, SHOOTER_VELOCITY).execute()
    kicker.kick_motor.set_voltage.assert_called_once_with(4.0)


def test_kicker_does_not_fire_when_below_target_velocity(
    shooter: ShooterSubsystem, kicker: KickerSubsystem, lights: PiLights
) -> None:
    shooter.shoot_encoder.get_velocity.return_value = 4000.0
    KickerShootWhenReadyCommand(shooter, kicker, lights, SHOOTER_VELOCITY).execute()
    kicker.kick_motor.set_voltage.assert_not_called()


def test_kicker_does_not_fire_at_exact_target_velocity(
    shooter: ShooterSubsystem, kicker: KickerSubsystem, lights: PiLights
) -> None:
    """Velocity gate is strict greater-than, so exact match does not fire."""
    shooter.shoot_encoder.get_velocity.return_value = 4500.0
    KickerShootWhenReadyCommand(shooter, kicker, lights, SHOOTER_VELOCITY).execute()
    kicker.kick_motor.set_voltage.assert_not_called()


def test_kicker_never_finishes(
    shooter: ShooterSubsystem, kicker: KickerSubsystem, lights: PiLights
) -> None:
    assert (
        KickerShootWhenReadyCommand(shooter, kicker, lights, None).isFinished() is False
    )


def test_kicker_end_stops_both_motors(
    shooter: ShooterSubsystem, kicker: KickerSubsystem, lights: PiLights
) -> None:
    KickerShootWhenReadyCommand(shooter, kicker, lights, None).end(interrupted=False)
    kicker.kick_motor.set_voltage.assert_called_once_with(0)
    shooter.shoot_motor.set_voltage.assert_called_once_with(0)
