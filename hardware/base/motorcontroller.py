from abc import ABC, abstractmethod
from typing import Any

from wpiutil import Sendable

from hardware.base import SendableABCMeta
from hardware.base.encoder import Encoder


class MotorController(Sendable, ABC, metaclass=SendableABCMeta):
    @abstractmethod
    def set_voltage(self, voltage: float) -> None: ...

    @abstractmethod
    def set_velocity(self, velocity: float) -> None: ...

    @abstractmethod
    def set_inverted(self, inverted: bool) -> None: ...

    def set_leader(self, leader: int, oppose: bool) -> None:
        raise Exception("Not Implemented")

    @abstractmethod
    def get_encoder(self) -> Encoder: ...

    @abstractmethod
    def get_voltage(self) -> float: ...

    @abstractmethod
    def get_forward_limit(self) -> bool: ...

    @abstractmethod
    def get_backward_limit(self) -> bool: ...

    @abstractmethod
    def get_motor_controller(self) -> Any: ...

    @abstractmethod
    def get_motor_id(self) -> int: ...

    @abstractmethod
    def disable(self) -> None: ...
