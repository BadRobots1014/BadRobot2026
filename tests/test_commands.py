"""Unit tests for robot commands."""

from unittest.mock import MagicMock, patch

import pytest

from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from commands.manual_extend_hopper import ManualExtendHopperCommand
from commands.run_intake import DUMP_VOLTAGE, INTAKE_VOLTAGE, RunIntakeCommand
from commands.shoot_kicker import KICKER_VOLTAGE, ShootKickerCommand
from commands.spin_shooter import SpinShooterCommand
from subsystems.climber import ClimberSubsystem
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.pilights import PiLights
from subsystems.shooter import SHOOTER_VELOCITY, ShooterSubsystem


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


@pytest.fixture
def climber() -> ClimberSubsystem:
    return ClimberSubsystem(MagicMock())


@pytest.fixture
def lights() -> PiLights:
    return MagicMock()


# --- ShootCommand ---


def test_shoot_command_sets_configured_velocity(shooter: ShooterSubsystem) -> None:
    SpinShooterCommand(shooter, 4000).execute()
    shooter.shoot_motor.set_velocity.assert_called_once_with(4000)


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
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.forward_limit_switch.get_state.return_value = True
    assert ManualExtendHopperCommand(hopper, lights, extend=True).isFinished() is True


def test_extend_not_finished_without_forward_limit(
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    assert ManualExtendHopperCommand(hopper, lights, extend=True).isFinished() is False


def test_retract_finished_when_backward_limit_hit(
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.backward_limit_switch.get_state.return_value = True
    assert ManualExtendHopperCommand(hopper, lights, extend=False).isFinished() is True


def test_retract_not_finished_without_backward_limit(
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.backward_limit_switch.get_state.return_value = False
    assert ManualExtendHopperCommand(hopper, lights, extend=False).isFinished() is False


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
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    hopper.backward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=False):
        ManualExtendHopperCommand(hopper, lights, extend=True).execute()
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output > 0


def test_extend_hopper_execute_retracts_with_negative_voltage(
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    hopper.backward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=False):
        ManualExtendHopperCommand(hopper, lights, extend=False).execute()
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output < 0


def test_extend_hopper_execute_uses_nt_when_test_mode_extend(
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.forward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=True):
        ManualExtendHopperCommand(hopper, lights, extend=True).execute()
    # set_extension_voltage_from_networktable calls left.set_voltage(extension_voltage)
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output == hopper.extension_voltage


def test_extend_hopper_execute_uses_nt_when_test_mode_retract(
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    hopper.backward_limit_switch.get_state.return_value = False
    with patch("robot.TEST_MODE_ENABLED", new=True):
        ManualExtendHopperCommand(hopper, lights, extend=False).execute()
    control = hopper.left_motor.set_control.call_args[0][0]
    assert control.output == -hopper.extension_voltage


# --- ExtendHopperCommand end() ---


def test_extend_hopper_end_stops_motor(
    hopper: HopperSubsystem, lights: PiLights
) -> None:
    ManualExtendHopperCommand(hopper, lights, extend=True).end(interrupted=False)
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
