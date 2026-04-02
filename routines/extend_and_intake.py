from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem


class ExtendAndIntakeRoutine(ParallelCommandGroup):
    """
    Extends the hopper and runs the intake.
    """

    def __init__(
        self, intake: IntakeSubsystem, hopper: HopperSubsystem,
    ):
        super().__init__()
        self.addCommands(
            SequentialCommandGroup(
                ExtendHopperCommand(hopper, extend=True),
                RunIntakeCommand(intake, dump=False),
            )
        )
