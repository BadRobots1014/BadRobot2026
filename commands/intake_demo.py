import commands2

from hardware.base.motor import Motor
from subsystems.intake import Intake

MOTOR_VOLTAGE = 2


class IntakeDemo(commands2.Command):
    def __init__(self, left: Motor, right: Motor, forward: bool):
        super().__init__()
        self.left = left
        self.right = right
        self.forward = forward

        self.right.set_inverted(True)

    def execute(self):
        if self.forward:
            self.right.set_voltage(MOTOR_VOLTAGE)
            self.left.set_voltage(MOTOR_VOLTAGE)
        else:
            self.right.set_voltage(-MOTOR_VOLTAGE)
            self.left.set_voltage(-MOTOR_VOLTAGE)

    def end(self, interrupted: bool):
        self.right.set_voltage(0)
        self.left.set_voltage(0)
