from hardware.base.encoder import Encoder

from abc import ABC

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class Motor(Sendable, ABC, metaclass=SendableABCMeta):

    def set_voltage(self, voltage: float):
        pass

    def set_velocity(self, voltage: float):
        pass

    def set_inverted(self, inverted: bool):
        pass

    def get_encoder(self) -> Encoder:
        pass

    def get_voltage(self) -> float:
        pass
