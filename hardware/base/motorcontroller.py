from __future__ import annotations

from abc import ABC, abstractmethod
import typing

from wpiutil import Sendable

from hardware.base import SendableABCMeta
from hardware.base.encoder import Encoder
from hardware.impl.motor_controller_config import MotorControllerConfig

if typing.TYPE_CHECKING:
    import phoenix6
    from rev import SparkBase


class MotorController(Sendable, ABC, metaclass=SendableABCMeta):
    @abstractmethod
    def set_voltage(self, voltage: float) -> None: ...

    @abstractmethod
    def set_velocity(self, velocity: float) -> None: ...

    @abstractmethod
    def set_inverted(self, inverted: bool) -> None: ...

    @abstractmethod
    def get_encoder(self) -> Encoder: ...

    @abstractmethod
    def get_encoder_position(self) -> float: ...

    @abstractmethod
    def get_voltage(self) -> float: ...

    @abstractmethod
    def get_forward_limit(self) -> bool: ...

    @abstractmethod
    def get_backward_limit(self) -> bool: ...

    @abstractmethod
    def get_inverted(self) -> bool: ...

    @abstractmethod
    def get_motor_controller(
        self,
    ) -> SparkBase | phoenix6.hardware.TalonFX: ...

    @abstractmethod
    def get_motor_id(self) -> int: ...

    @abstractmethod
    def zero_relative_encoder(self) -> None: ...

    @abstractmethod
    def apply_configs(self, motor_controller_config: MotorControllerConfig) -> None: ...

    @abstractmethod
    def disable(self) -> None: ...
