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

    def __init__(
        self,
        inverted: bool = False,
        idle_mode: MotorControllerIdleMode = MotorControllerIdleMode.BRAKE,
        leader: "MotorController | None" = None,
    ):
        self.inverted = inverted
        self.idle_mode = idle_mode
        self.leader = leader
