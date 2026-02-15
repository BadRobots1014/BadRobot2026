from abc import ABC

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class LEDController(Sendable, ABC, metaclass=SendableABCMeta):
    def get_solid(self, r: int, g: int, b: int):
        pass

    def get_rainbow(self, saturation: int, value: int, speed: int):
        pass

    def get_gradient(self, continuous: bool, colors: list[tuple]):
        pass

    def apply_pattern(self, pattern):
        pass
