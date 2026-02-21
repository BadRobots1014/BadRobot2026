from abc import ABC
from enum import Enum

from wpiutil import Sendable

from hardware.base import SendableABCMeta
from hardware.base.encoder import Encoder
from rev import SparkBase


class IdleMode(Enum):
    kBrake = 0
    kCoast = 1

class Motor(Sendable, ABC, metaclass=SendableABCMeta):
    def set_voltage(self, voltage: float) -> None:
        pass

    def set_velocity(self, velocity: float) -> None:
        pass

    def set_inverted(self, inverted: bool) -> None:
        pass

    def get_inverted(self, inverted: bool) -> None:
        pass

    def get_encoder(self) -> Encoder:
        pass

    def get_voltage(self) -> float:
        pass

    def get_forward_limit(self) -> bool:
        pass

    def get_backward_limit(self) -> bool:
        pass

    def get_motor_controller(self) -> SparkBase:
        pass

    def disable(self) -> None:
        pass

    def set_idle_mode(self, idle_mode: IdleMode) -> None:
        pass
