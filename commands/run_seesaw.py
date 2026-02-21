import commands2

from subsystems.intake import Intake

MOTOR_VOLTAGE = 4


class RunSeesaw(commands2.Command):
    def __init__(self, intake: Intake, dump: bool = True):
        super().__init__()
        self.intake = intake
        self.dump = dump

        if dump:
            self.intake.set_seesaw_voltage(MOTOR_VOLTAGE)
        else:
            self.intake.set_seesaw_voltage(-MOTOR_VOLTAGE)

    def isFinished(self) -> bool:
        if (self.intake.seesaw_forward_extended()) or (
            self.intake.seesaw_backward_extended()
        ):
            self.intake.set_seesaw_voltage(0)
            return True

        return False
