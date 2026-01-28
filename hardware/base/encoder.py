from abc import ABC

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class Encoder(Sendable, ABC, metaclass=SendableABCMeta):
    def get_velocity(self) -> float:
        return 0.0

    def get_position(self) -> float:
        return 0.0

    def set_position(self, position: float):
        pass
