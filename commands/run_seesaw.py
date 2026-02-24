import commands2

from subsystems.seesaw import Seesaw

MOTOR_VOLTAGE = 4


class RunSeesaw(commands2.Command):
    def __init__(self, seesaw: Seesaw, dump: bool = True):
        super().__init__()
        self.seesaw = seesaw
        self.dump = dump

        if dump:
            self.seesaw.set_seesaw_voltage(MOTOR_VOLTAGE)
        else:
            self.seesaw.set_seesaw_voltage(-MOTOR_VOLTAGE)

    def isFinished(self) -> bool:
        if (self.seesaw.seesaw_forward_extended()) or (
            self.seesaw.seesaw_backward_extended()
        ):
            self.seesaw.set_seesaw_voltage(0)
            return True

        return False
