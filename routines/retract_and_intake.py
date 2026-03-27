from commands2 import ParallelCommandGroup

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.pilights import PiLights


class RetractInRoutine(ParallelCommandGroup):
    """
    Retract the hopper and runs the intake.
    """

    def __init__(
        self, intake: IntakeSubsystem, hopper: HopperSubsystem, lights: PiLights
    ):
        super().__init__()
        self.addCommands(
            ParallelCommandGroup(
                ExtendHopperCommand(hopper, lights, extend=False),
                RunIntakeCommand(intake, dump=False),
            )
        )
