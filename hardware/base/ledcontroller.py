from abc import ABC, abstractmethod

from wpilib import LEDPattern


class LEDController(ABC):
    @abstractmethod
    def get_solid(self, r: int, g: int, b: int) -> LEDPattern: ...

    @abstractmethod
    def get_rainbow(self, saturation: int, value: int, speed: float) -> LEDPattern: ...

    @abstractmethod
    def get_gradient(self, continuous: bool, colors: list[tuple]) -> LEDPattern: ...

    @abstractmethod
    def apply_pattern(self, pattern: LEDPattern) -> None: ...
