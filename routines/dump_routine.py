from commands2 import (
    ParallelCommandGroup,
    RepeatCommand,
    SequentialCommandGroup,
    WaitCommand,
)

from commands.manual_extend_hopper import ManualExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from commands.run_kicker import RunKickerCommand
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.pilights import PiLights


class DumpRoutine(SequentialCommandGroup):
    """
    Extends the hopper and runs the intake. Can be used to intake or dump depending on dump argument(False to intake, True to dump)
    """

    def __init__(
        self,
        intake: IntakeSubsystem,
        hopper: HopperSubsystem,
        kicker: KickerSubsystem,
        lights: PiLights,
    ):
        super().__init__()
        self.addCommands(
            ManualExtendHopperCommand(hopper, lights, extend=True),
            ParallelCommandGroup(
                RunIntakeCommand(intake, dump=True),
                RepeatCommand(
                    SequentialCommandGroup(
                        RunKickerCommand(kicker, invert=False).withTimeout(0.2),
                        WaitCommand(0.2),
                        RunKickerCommand(kicker, invert=True).withTimeout(0.2),
                        WaitCommand(0.2),
                    )
                ),
            ),
        )
