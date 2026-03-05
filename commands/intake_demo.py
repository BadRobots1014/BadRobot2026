import commands2

from hardware.base.motorcontroller import MotorController
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)

MOTOR_VOLTAGE = 3


class IntakeDemoCommand(commands2.Command):
    def __init__(self, left: MotorController, right: MotorController, forward: bool):
        super().__init__()
        self.left = left
        self.right = right
        self.forward = forward

        right_config = MotorControllerConfig(
            inverted=True,
            idle_mode=MotorControllerIdleMode.BRAKE,
            pidf=(0, 0, 0, 0),
            leader=left,
        )
        self.right.apply_configs(right_config)

    def execute(self) -> None:
        if self.forward:
            self.right.set_voltage(MOTOR_VOLTAGE)
            self.left.set_voltage(MOTOR_VOLTAGE)
        else:
            self.right.set_voltage(-MOTOR_VOLTAGE)
            self.left.set_voltage(-MOTOR_VOLTAGE)

    def end(self, interrupted: bool) -> None:
        self.right.set_voltage(0)
        self.left.set_voltage(0)
