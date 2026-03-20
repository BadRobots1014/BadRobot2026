from commands2 import ParallelCommandGroup

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from subsystems.pilights import PiLights
from subsystems.talonFXIntake import TalonIntakeSubsystem


class RetractIn(ParallelCommandGroup):
    """
    Extends the hopper and runs the intake. Can be used to intake or dump depending on dump argument(False to intake, True to dump)
    """

    def __init__(self, intake: TalonIntakeSubsystem, lights: PiLights):
        super().__init__()
        self.addCommands(
            ParallelCommandGroup(
                ExtendHopperCommand(intake, lights, extend=False),
                RunIntakeCommand(intake, dump=False),
            )
        )
