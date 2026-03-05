from typing import TYPE_CHECKING, List
from enum import Enum

if TYPE_CHECKING:
    from hardware.base.motorcontroller import MotorController


class MotorControllerIdleMode(Enum):
    COAST = 0
    BRAKE = 1


class MotorControllerConfig:
    inverted: bool
    idle_mode: MotorControllerIdleMode
    leader: "MotorController | None"
    pidf: list[float]

    def __init__(
        self,
        inverted: bool = False,
        idle_mode: MotorControllerIdleMode = MotorControllerIdleMode.BRAKE,
            pidf = list[float], # list of p i d and f
        leader: "MotorController | None" = None,
    ):
        if pidf is None:
            pidf: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.inverted = inverted
        self.idle_mode = idle_mode
        self.leader = leader
        self.pidf = pidf
