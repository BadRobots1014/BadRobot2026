from commands2 import (
    ParallelCommandGroup,
    RepeatCommand,
    SequentialCommandGroup,
    WaitCommand,
)

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from commands.shoot_kicker import ShootKickerCommand
from subsystems.kicker import KickerSubsystem
from subsystems.pilights import PiLights
from subsystems.talonFXIntake import TalonIntakeSubsystem


class DumpRoutine(SequentialCommandGroup):
    """
    Extends the hopper and runs the intake. Can be used to intake or dump depending on dump argument(False to intake, True to dump)
    """

    def __init__(
        self, intake: TalonIntakeSubsystem, kicker: KickerSubsystem, lights: PiLights
    ):
        super().__init__()
        self.addCommands(
            ExtendHopperCommand(intake, lights, extend=True),
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
