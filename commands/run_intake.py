import commands2
from subsystems.intake import IntakeSubsystem

INTAKE_VOLTAGE = 3000


class RunIntakeCommand(commands2.Command):
    def __init__(self, intake: IntakeSubsystem, dump: bool):
        super().__init__()
        self.addRequirements(intake)
        self.intake = intake
        self.dump = dump

    def execute(self):
        if self.dump:
            self.intake.set_intake_voltage(-12)
        else:
            self.intake.set_intake_voltage(4.5)


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.intake.set_intake_voltage(0)
