from commands2 import Subsystem

from hardware.base.ledcontroller import LEDController


class LightSubsystem(Subsystem):
    def __init__(self, controller: LEDController):
        super().__init__()
        self.controller = controller

        self.defult_pattern = self.controller.get_solid(255, 255, 0)
        self.current_pattern = None

        self.set_default()

    def set_solid(self, r: int, g: int, b: int) -> None:
        self.current_pattern = self.controller.get_solid(r, g, b)

    def set_rainbow(self, saturation: int, value: int, speed: int) -> None:
        self.current_pattern = self.controller.get_rainbow(saturation, value, speed)

    def set_gradient(self, continuous: bool, colors: list[tuple]) -> None:
        self.current_pattern = self.controller.get_gradient(continuous, colors)

    def set_default(self) -> None:
        self.current_pattern = self.defult_pattern

    def periodic(self) -> None:
        if self.current_pattern is None:
            self.set_default()
        else:
            self.controller.apply_pattern(self.current_pattern)
