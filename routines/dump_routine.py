from commands2 import (
    ParallelCommandGroup,
)

from commands.run_conveyor import RunConveyor
from commands.run_intake import RunIntakeCommand
from commands.run_kicker import RunKickerCommand
from subsystems.conveyor import ConveyorSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem


class DumpRoutine(ParallelCommandGroup):
    """
    Extends the hopper and runs the intake. Can be used to intake or dump depending on dump argument(False to intake, True to dump)
    """

    def __init__(
        self,
        intake: IntakeSubsystem,
        kicker: KickerSubsystem,
        conveyor: ConveyorSubsystem,
    ):
        super().__init__()
        self.addCommands(
            RunIntakeCommand(intake, dump=True),
            RunKickerCommand(kicker, invert=True),
            RunConveyor(conveyor, shoot_direction=False),
        )
