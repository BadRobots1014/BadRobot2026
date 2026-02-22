from commands2 import Subsystem
from hardware.base.motor import Motor
from hardware.base.switch import LimitSwitch

# Dumping velocity should be 1500


class Intake(Subsystem):
    def __init__(
        self,
        intake: Motor,
        left: Motor,
        right: Motor,
        forward: LimitSwitch,
        backward: LimitSwitch,
    ) -> None:
        super().__init__()
        self.intake_motor = intake

        self.left = left
        self.right = right
        self.right.set_leader(self.left.get_motor_id(), True)

        self.forward = forward
        self.backward = backward

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
        return self.forward.get_state()

    def backward_extended(self) -> bool:
        return self.backward.get_state()
