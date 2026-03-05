import commands2

from subsystems.intake import IntakeSubsystem

INTAKE_VOLTAGE = 4.5
DUMP_VOLTAGE = -5


class RunIntakeCommand(commands2.Command):
    def __init__(self, intake: IntakeSubsystem, dump: bool):
        super().__init__()
        self.addRequirements(intake)
        self.intake = intake
        self.dump = dump

    def execute(self) -> None:
        if self.dump:
            self.intake.set_intake_voltage(DUMP_VOLTAGE)
        else:
            self.intake.set_intake_voltage(INTAKE_VOLTAGE)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.intake.set_intake_voltage(0)
