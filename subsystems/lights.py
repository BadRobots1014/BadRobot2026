from commands2 import Subsystem

from hardware.base.ledcontroller import LEDController

BRIGHTNESS_MULTIPLIER = 0.4


class LightSubsystem(Subsystem):
    def __init__(self, controller: LEDController):
        super().__init__()
        self.controller = controller

        self.default_pattern = self.controller.get_solid(255, 255, 0).atBrightness(
            BRIGHTNESS_MULTIPLIER
        )
        self.current_pattern = None

        self.set_default()

    def set_solid(self, r: int, g: int, b: int) -> None:
        self.current_pattern = self.controller.get_solid(r, g, b).atBrightness(
            BRIGHTNESS_MULTIPLIER
        )

    def set_rainbow(self, saturation: int, value: int, speed: int) -> None:
        self.current_pattern = self.controller.get_rainbow(
            saturation, value, speed
        ).atBrightness(BRIGHTNESS_MULTIPLIER)

    def set_gradient(self, continuous: bool, colors: list[tuple]) -> None:
        self.current_pattern = self.controller.get_gradient(
            continuous, colors
        ).atBrightness(BRIGHTNESS_MULTIPLIER)

    def set_default(self) -> None:
        self.current_pattern = self.default_pattern

    def periodic(self) -> None:
        if self.current_pattern is None:
            self.set_default()
        else:
            self.controller.apply_pattern(self.current_pattern)
