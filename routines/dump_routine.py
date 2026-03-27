from commands.extend_hopper import ExtendHopperCommand
from commands2 import (
    ParallelCommandGroup,
    RepeatCommand,
    SequentialCommandGroup,
    WaitCommand,
)

from commands.run_intake import RunIntakeCommand
from commands.shoot_kicker import ShootKickerCommand
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
            ExtendHopperCommand(hopper, lights, extend=True),
            ParallelCommandGroup(
                RunIntakeCommand(intake, dump=True),
                RepeatCommand(
                    SequentialCommandGroup(
                        ShootKickerCommand(kicker, invert=False).withTimeout(0.2),
                        WaitCommand(0.2),
                        ShootKickerCommand(kicker, invert=True).withTimeout(0.2),
                        WaitCommand(0.2),
                    )
                ),
            ),
        )
