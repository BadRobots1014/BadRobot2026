from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.manual_extend_hopper import ManualExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.pilights import PiLights


class ExtendAndIntakeRoutine(ParallelCommandGroup):
    """
    Extends the hopper and runs the intake.
    """

    def __init__(
        self, intake: IntakeSubsystem, hopper: HopperSubsystem, lights: PiLights
    ):
        super().__init__()
        self.addCommands(
            SequentialCommandGroup(
                ManualExtendHopperCommand(hopper, lights, extend=True),
                RunIntakeCommand(intake, dump=False),
            )
        )
