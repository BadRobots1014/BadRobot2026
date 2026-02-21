from commands2 import Subsystem

from hardware.base.motor import Motor

# Dumping velocity should be 1500


class Seesaw(Subsystem):
    def __init__(self, seesaw: Motor) -> None:
        super().__init__()
        self.seesaw_motor = seesaw

    def set_seesaw_voltage(self, voltage: float):
        self.seesaw_motor.set_voltage(voltage)

    @property
    def seesaw_voltage(self):
        return self.seesaw_motor.get_voltage()

    def seesaw_forward_extended(self):
        return self.seesaw_motor.get_forward_limit()  # have to add spark max limits

    def seesaw_backward_extended(self):
        return self.seesaw_motor.get_backward_limit()  # have to add spark max limits
