from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from hardware.base.motorcontroller import MotorController


class MotorControllerIdleMode(Enum):
    COAST = 0
    BRAKE = 1


class MotorControllerConfig:
    inverted: bool
    idle_mode: MotorControllerIdleMode
    leader: "MotorController | None"
    pidf: tuple[float, float, float, float]

    def __init__(
        self,
        inverted: bool = False,
        idle_mode: MotorControllerIdleMode = MotorControllerIdleMode.BRAKE,
        pidf: tuple[float, float, float, float] = (0, 0, 0, 0),
        leader: "MotorController | None" = None,
    ):
        self.inverted = inverted
        self.idle_mode = idle_mode
        self.leader = leader
        self.pidf = pidf
