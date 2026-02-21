from commands2 import Subsystem

from hardware.base.motor import Motor

# Dumping velocity should be 1500


class Intake(Subsystem):
    def __init__(self, intake: Motor, left: Motor, right: Motor,) -> None:
        super().__init__()
        self.intake_motor = intake
        self.left = left
        self.right = right
        self.right.set_leader(self.left.get_motor_id())

    def set_intake_voltage(self, voltage: float):
        self.intake_motor.set_voltage(voltage)

    def set_extension_voltage(self, voltage: float):
        self.left.set_voltage(voltage)

    @property
    def intake_voltage(self):
        return self.intake_motor.get_voltage()

    @property
    def extension_voltage(self):
        return self.left.get_voltage()

    def forward_extended(self) -> bool:
        return self.extension_motor.get_forward_limit()

    def backward_extended(self) -> bool:
        return self.extension_motor.get_backward_limit()
