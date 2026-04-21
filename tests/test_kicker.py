"""Unit tests for KickerSubsystem."""

from unittest.mock import MagicMock

import pytest

from subsystems.kicker import (
    KICKER_DUMP_VOLTAGE,
    KICKER_SHOOT_VOLTAGE,
    KickerSubsystem,
)


@pytest.fixture
def kicker() -> KickerSubsystem:
    return KickerSubsystem(MagicMock(), MagicMock())


def test_default_kick_shoot_voltage(kicker: KickerSubsystem) -> None:
    assert kicker.kick_shoot_voltage == KICKER_SHOOT_VOLTAGE


def test_default_kick_dump_voltage(kicker: KickerSubsystem) -> None:
    assert kicker.kick_dump_voltage == KICKER_DUMP_VOLTAGE


def test_set_kick_voltage(kicker: KickerSubsystem) -> None:
    kicker.set_kick_voltage(3.0)
    kicker.kick_motor.set_voltage.assert_called_once_with(3.0)


def test_set_kick_shoot_voltage_from_nt(kicker: KickerSubsystem) -> None:
    kicker.kick_shoot_voltage = 5.0
    kicker.set_kick_shoot_voltage_from_networktables()
    kicker.kick_motor.set_voltage.assert_called_once_with(KICKER_SHOOT_VOLTAGE)


def test_set_kick_dump_voltage_from_nt_uses_dump_not_shoot(
    kicker: KickerSubsystem,
) -> None:
    """Regression: was using kick_shoot_voltage instead of kick_dump_voltage."""
    kicker.kick_shoot_voltage = 5.0
    kicker.kick_dump_voltage = 2.0
    kicker.set_kick_dump_voltage_from_networktables()
    kicker.kick_motor.set_voltage.assert_called_once_with(2.0)


def test_kick_distance_reads_encoder(kicker: KickerSubsystem) -> None:
    kicker.kick_encoder.get_position.return_value = 3.2
    assert kicker.kick_distance == 3.2
