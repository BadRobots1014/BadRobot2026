from abc import ABC, abstractmethod

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class Encoder(Sendable, ABC, metaclass=SendableABCMeta):
    @abstractmethod
    def get_velocity(self) -> float: ...

    @abstractmethod
    def get_position(self) -> float: ...

    @abstractmethod
    def set_position(self, position: float) -> None: ...
