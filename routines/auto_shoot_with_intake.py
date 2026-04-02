from commands2 import RepeatCommand, WaitCommand

from commands.run_intake import RunIntakeCommand
from subsystems.intake import IntakeSubsystem


class AutoShootWithIntake(RepeatCommand):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__(
            RunIntakeCommand(intake, dump=False)
            .withTimeout(0.5)
            .andThen(WaitCommand(0.3))
            .andThen(RunIntakeCommand(intake, dump=False).withTimeout(0.5))
            .andThen(WaitCommand(0.3))
        )
