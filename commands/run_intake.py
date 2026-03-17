import commands2

import robot
from subsystems.talonFXIntake import TalonIntakeSubsystem

INTAKE_VOLTAGE = 4.5
DUMP_VOLTAGE = -5


class RunIntakeCommand(commands2.Command):
    def __init__(self, intake: TalonIntakeSubsystem, dump: bool):
        super().__init__()
        self.addRequirements(intake)
        self.intake = intake
        self.dump = dump

    def execute(self) -> None:
        if not self.dump:
            if robot.TEST_MODE_ENABLED:
                self.intake.set_intake_voltage_from_networktable()
            else:
                self.intake.set_intake_voltage(INTAKE_VOLTAGE)
        elif robot.TEST_MODE_ENABLED:
            self.intake.set_dump_voltage_from_networktable()
        else:
            self.intake.set_intake_voltage(DUMP_VOLTAGE)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.intake.set_intake_voltage(0)
