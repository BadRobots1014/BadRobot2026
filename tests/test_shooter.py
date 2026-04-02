"""Unit tests for ShooterSubsystem."""

from unittest.mock import MagicMock

import pytest

from subsystems.shooter import (
    SHOOTER_VELOCITY,
    ShooterSubsystem,
)


@pytest.fixture
def shooter() -> ShooterSubsystem:
    return ShooterSubsystem(MagicMock(), MagicMock(), MagicMock())


def test_default_shoot_velocity(shooter: ShooterSubsystem) -> None:
    assert shooter.shoot_velocity == SHOOTER_VELOCITY


def test_set_shoot_voltage(shooter: ShooterSubsystem) -> None:
    shooter.set_shoot_voltage(6.0)
    shooter.shoot_motor.set_voltage.assert_called_once_with(6.0)


def test_set_shoot_velocity(shooter: ShooterSubsystem) -> None:
    shooter.set_shoot_velocity(3000.0)
    shooter.shoot_motor.set_velocity.assert_called_once_with(3000.0)


def test_get_shoot_velocity_reads_encoder(shooter: ShooterSubsystem) -> None:
    shooter.shoot_encoder.get_velocity.return_value = 4200.0
    assert shooter.get_shoot_velocity() == 4200.0


def test_shoot_distance_reads_encoder(shooter: ShooterSubsystem) -> None:
    shooter.shoot_encoder.get_position.return_value = 10.5
    assert shooter.shoot_distance == 10.5
