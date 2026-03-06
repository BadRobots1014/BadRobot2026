import typing
from typing import TYPE_CHECKING
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
    p: float
    i: float
    d: float
    f: float

    def __init__(
        self,
        inverted: bool = False,
        idle_mode: MotorControllerIdleMode = MotorControllerIdleMode.BRAKE,
        leader: "MotorController | None" = None,
        p: float = 0,
        i: float = 0,
        d: float = 0,
        f: float = 0,
    ):
        self.p = p
        self.i = i
        self.d = d
        self.f = f
        self.inverted = inverted
        self.idle_mode = idle_mode
        self.leader = leader
