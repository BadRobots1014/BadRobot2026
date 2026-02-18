from commands2 import Subsystem

from hardware.base.motor import Motor

# Dumping velocity should be 1500


class Intake(Subsystem):
    def __init__(self, intake: Motor, extension: Motor) -> None:
        super().__init__()
        self.intake_motor = intake
        self.extension_motor = extension

    def set_intake_voltage(self, voltage: float):
        self.intake_motor.set_voltage(voltage)

    def set_extension_voltage(self, voltage: float):
        self.extension_motor.set_voltage(voltage)

    @property
    def intake_voltage(self):
        return self.intake_motor.get_voltage()

    @property
    def extension_voltage(self):
        return self.extension_motor.get_voltage()

    # Don't make a property
    def forward_extended(self) -> bool:
        return self.extension_motor.get_forward_limit()

    def backward_extended(self) -> bool:
        return self.extension_motor.get_backward_limit()
