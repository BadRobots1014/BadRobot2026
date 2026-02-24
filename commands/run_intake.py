import commands2
from subsystems.intake import IntakeSubsystem

INTAKE_VOLTAGE = 4.0


class RunIntakeCommand(commands2.Command):
    def __init__(self, intake: IntakeSubsystem, dump: bool):
        super().__init__()
        self.addRequirements(intake)
        self.intake = intake
        if dump:
            self.intake.set_intake_voltage(-INTAKE_VOLTAGE)
        else:
            self.intake.set_intake_voltage(INTAKE_VOLTAGE)

    def end(self, interrupted: bool):
        self.intake.set_intake_voltage(0)
