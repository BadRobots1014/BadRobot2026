from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.pilights import PiLights


class ExtendAndIntakeRoutine(ParallelCommandGroup):
    """
    Extends the hopper and runs the intake. Can be used to intake or dump depending on dump argument(False to intake, True to dump)
    """

    def __init__(
        self, intake: IntakeSubsystem, hopper: HopperSubsystem, lights: PiLights
    ):
        super().__init__()
        self.addCommands(
            SequentialCommandGroup(
                ExtendHopperCommand(hopper, lights, extend=True),
                RunIntakeCommand(intake, dump=False),
            )
        )
