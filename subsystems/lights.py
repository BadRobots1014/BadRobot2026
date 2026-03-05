from commands2 import Subsystem
from hardware.base.ledcontroller import LEDController


class LightSubsystem(Subsystem):
    def __init__(self, controller: LEDController):
        super().__init__()
        self.controller = controller

        self.current_pattern = None

    def set_solid(self, r: int, g: int, b: int) -> None:
        self.current_pattern = self.controller.get_solid(r, g, b)

    def set_rainbow(self, saturation: int, value: int, speed: int) -> None:
        self.current_pattern = self.controller.get_rainbow(saturation, value, speed)

    def set_gradient(self, continuous: bool, colors: list[tuple]) -> None:
        self.current_pattern = self.controller.get_gradient(continuous, colors)

    def periodic(self) -> None:
        if self.current_pattern is None:
            self.set_solid(255, 255, 255)
        else:
            self.controller.apply_pattern(self.current_pattern)
