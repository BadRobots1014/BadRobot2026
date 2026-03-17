from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from subsystems.talonFXIntake import TalonIntakeSubsystem


class ExtendAndIntakeRoutine(ParallelCommandGroup):
    """
    Extends the hopper and runs the intake. Can be used to intake or dump depending on dump argument(False to intake, True to dump)
    """

    def __init__(self, intake: TalonIntakeSubsystem):
        super().__init__()
        self.addCommands(
            SequentialCommandGroup(
                ExtendHopperCommand(intake, extend=True),
                RunIntakeCommand(intake, dump=False),
            )
        )
