from commands2 import Subsystem
from hardware.base.motor import Motor


class Intake(Subsystem):
    def __init__(self, intake: Motor, extension: Motor) -> None:
        super().__init__()
        self.intake = intake
        self.extension = extension

    def set_intake_voltage(self, voltage: float):
        self.intake.set_voltage(voltage)

    def set_extension_voltage(self, voltage: float):
        self.extension.set_voltage(voltage)

    @property
    def intake_voltage(self):
        return self.intake.get_voltage()

    @property
    def extension_voltage(self):
        return self.extension.get_voltage()

    def forward_extended(self) -> bool:
        return self.extension.get_forward_limit()

    def backward_extended(self) -> bool:
        return self.extension.get_backward_limit()
