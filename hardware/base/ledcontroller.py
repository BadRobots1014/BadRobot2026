from abc import ABC, abstractmethod

from wpilib import LEDPattern

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class LEDController(Sendable, ABC, metaclass=SendableABCMeta):
    @abstractmethod
    def get_solid(self, r: int, g: int, b: int):
        pass

    @abstractmethod
    def get_rainbow(self, saturation: int, value: int, speed: float):
        pass

    @abstractmethod
    def get_gradient(self, continuous: bool, colors: list[tuple]):
        pass

    @abstractmethod
    def apply_pattern(self, pattern: LEDPattern):
        pass
