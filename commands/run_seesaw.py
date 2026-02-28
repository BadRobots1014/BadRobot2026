import commands2

from subsystems.seesaw import SeesawSubsystem

MOTOR_VOLTAGE = 2


class RunSeesawCommand(commands2.Command):
    def __init__(self, seesaw: SeesawSubsystem, dump: bool = True):
        super().__init__()
        self.seesaw = seesaw
        self.dump = dump

    def execute(self):
        if self.dump:
            self.seesaw.set_seesaw_voltage(MOTOR_VOLTAGE)
        else:
            self.seesaw.set_seesaw_voltage(-MOTOR_VOLTAGE)

    # def isFinished(self) -> bool:
    #     if (self.seesaw.seesaw_forward_extended()) or (
    #         self.seesaw.seesaw_backward_extended()
    #     ):
    #         self.seesaw.set_seesaw_voltage(0)
    #         return True
    #
    #     return False

    def end(self, inter: bool):
        self.seesaw.set_seesaw_voltage(0)
