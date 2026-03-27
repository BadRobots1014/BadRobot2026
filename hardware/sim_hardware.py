# Physical hardware / configuration that is patched out in simulation

from wpilib import Color, LEDPattern

from hardware.base.ledcontroller import LEDController
from hardware.base.switch import LimitSwitch


class DummyLED(LEDController):
    """Non-functional LED Controller for Simulation"""

    def __init__(self, _port: int, length: int):  # noqa:ARG002
        super().__init__()

        self.spacing = 0.015

        print("Lights initialized")

    def get_solid(self, r: int, g: int, b: int) -> LEDPattern:
        return LEDPattern.solid(Color(r, g, b))

    def get_rainbow(self, saturation: int, value: int, speed: float) -> LEDPattern:
        return LEDPattern.rainbow(saturation, value).scrollAtAbsoluteSpeed(
            speed, self.spacing
        )

    def get_gradient(self, continuous: bool, colors: list[tuple]) -> LEDPattern:
        return LEDPattern.gradient(
            (
                LEDPattern.GradientType.kContinuous
                if continuous
                else LEDPattern.GradientType.kDiscontinuous
            ),
            [Color(color[0], color[1], color[2]) for color in colors],
        )

    def apply_pattern(self, pattern: LEDPattern) -> None:  # noqa:ARG002
        return None


class DummyLimitSwitch(LimitSwitch):
    def __init__(self, default_state: bool = False):
        self.default_state = default_state

    def get_state(self) -> bool:
        return self.default_state
