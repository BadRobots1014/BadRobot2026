from __future__ import annotations
from typing import TYPE_CHECKING
from enum import Enum

if TYPE_CHECKING:
     from hardware.base.motor import Motor

class MotorControllerIdleMode(Enum):
    COAST = 0
    BRAKE = 1


class MotorControllerConfig:
    inverted: bool
    idle_mode: MotorControllerIdleMode
    leader: "Motor | None"

    def __init__(
        self,
        inverted: bool = False,
        idle_mode: MotorControllerIdleMode = MotorControllerIdleMode.BRAKE,
        leader: "Motor | None" = None,
    ):
        self.inverted = inverted
        self.idle_mode = idle_mode
        self.leader = leader
