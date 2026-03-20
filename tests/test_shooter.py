"""Unit tests for ShooterSubsystem."""

from unittest.mock import MagicMock

import pytest

from subsystems.shooter import (
    SHOOTER_VELOCITY,
    ShooterSubsystem,
)

from subsystems.kicker import (
    KICKER_DUMP_VOLTAGE,
    KICKER_SHOOT_VOLTAGE,
    KickerSubsystem
)


@pytest.fixture
def shooter() -> ShooterSubsystem:
    return ShooterSubsystem(
        MagicMock(), MagicMock(), MagicMock(),
    )

@pytest.fixture
def kicker() -> KickerSubsystem:
    return KickerSubsystem(
        MagicMock(), MagicMock()
    )


def test_default_shoot_velocity(shooter: ShooterSubsystem) -> None:
    assert shooter.shoot_velocity == SHOOTER_VELOCITY


def test_default_kick_shoot_voltage(kicker: KickerSubsystem) -> None:
    assert kicker.kick_shoot_voltage == KICKER_SHOOT_VOLTAGE


def test_default_kick_dump_voltage(kicker: KickerSubsystem) -> None:
    assert kicker.kick_dump_voltage == KICKER_DUMP_VOLTAGE


def test_set_shoot_voltage(shooter: ShooterSubsystem) -> None:
    shooter.set_shoot_voltage(6.0)
    shooter.shoot_motor.set_voltage.assert_called_once_with(6.0)


def test_set_shoot_velocity(shooter: ShooterSubsystem) -> None:
    shooter.set_shoot_velocity(3000.0)
    shooter.shoot_motor.set_velocity.assert_called_once_with(3000.0)


def test_set_kick_voltage(kicker: KickerSubsystem) -> None:
    kicker.set_kick_voltage(3.0)
    kicker.kick_motor.set_voltage.assert_called_once_with(3.0)


def test_set_kick_shoot_voltage_from_nt(kicker: KickerSubsystem) -> None:
    kicker.kick_shoot_voltage = 5.0
    kicker.set_kick_shoot_voltage_from_networktables()
    kicker.kick_motor.set_voltage.assert_called_once_with(5.0)


def test_set_kick_dump_voltage_from_nt_uses_dump_not_shoot(
    shooter: ShooterSubsystem, kicker: KickerSubsystem
) -> None:
    """Regression: was using kick_shoot_voltage instead of kick_dump_voltage."""
    shooter.kick_shoot_voltage = 5.0
    kicker.kick_dump_voltage = 2.0
    kicker.set_kick_dump_voltage_from_networktables()
    kicker.kick_motor.set_voltage.assert_called_once_with(2.0)


def test_get_shoot_velocity_reads_encoder(shooter: ShooterSubsystem) -> None:
    shooter.shoot_encoder.get_velocity.return_value = 4200.0
    assert shooter.get_shoot_velocity() == 4200.0


def test_shoot_distance_reads_encoder(shooter: ShooterSubsystem) -> None:
    shooter.shoot_encoder.get_position.return_value = 10.5
    assert shooter.shoot_distance == 10.5


def test_kick_distance_reads_encoder(shooter: ShooterSubsystem, kicker: KickerSubsystem) -> None:
    kicker.kick_encoder.get_position.return_value = 3.2
    assert kicker.kick_distance == 3.2
