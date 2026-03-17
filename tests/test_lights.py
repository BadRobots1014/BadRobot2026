"""Unit tests for LightSubsystem."""

from unittest.mock import MagicMock

import pytest

from subsystems.lights import BRIGHTNESS_MULTIPLIER, LightSubsystem


@pytest.fixture
def lights() -> LightSubsystem:
    controller = MagicMock()
    subsystem = LightSubsystem(controller)
    controller.reset_mock()  # Clear calls made during __init__
    return subsystem


def test_periodic_applies_current_pattern(lights: LightSubsystem) -> None:
    pattern = MagicMock()
    lights.current_pattern = pattern
    lights.periodic()
    lights.controller.apply_pattern.assert_called_once_with(pattern)


def test_periodic_resets_to_default_when_pattern_is_none(
    lights: LightSubsystem,
) -> None:
    lights.current_pattern = None
    lights.periodic()
    # set_default() was called; apply_pattern is not called in the same periodic tick
    lights.controller.apply_pattern.assert_not_called()
    assert lights.current_pattern is lights.default_pattern


def test_set_default_restores_default_pattern(lights: LightSubsystem) -> None:
    lights.current_pattern = MagicMock()
    lights.set_default()
    assert lights.current_pattern is lights.default_pattern


def test_set_solid_updates_current_pattern(lights: LightSubsystem) -> None:
    lights.set_solid(255, 0, 128)
    lights.controller.get_solid.assert_called_once_with(255, 0, 128)
    lights.controller.get_solid.return_value.atBrightness.assert_called_once_with(
        BRIGHTNESS_MULTIPLIER
    )
    assert (
        lights.current_pattern
        is lights.controller.get_solid.return_value.atBrightness.return_value
    )


def test_set_rainbow_updates_current_pattern(lights: LightSubsystem) -> None:
    lights.set_rainbow(100, 150, 10)
    lights.controller.get_rainbow.assert_called_once_with(100, 150, 10)
    assert (
        lights.current_pattern
        is lights.controller.get_rainbow.return_value.atBrightness.return_value
    )


def test_set_gradient_updates_current_pattern(lights: LightSubsystem) -> None:
    continuous = True
    colors = [(255, 0, 0), (0, 0, 255)]
    lights.set_gradient(continuous=continuous, colors=colors)
    lights.controller.get_gradient.assert_called_once_with(continuous, colors)
    assert (
        lights.current_pattern
        is lights.controller.get_gradient.return_value.atBrightness.return_value
    )
