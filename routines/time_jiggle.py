from commands2 import RepeatCommand, SequentialCommandGroup, WaitCommand

from commands.manual_extend_hopper import ManualExtendHopperCommand
from subsystems.pilights import PiLights
from subsystems.talonFXIntake import TalonIntakeSubsystem


class TimeJiggle(RepeatCommand):
    def __init__(self, talon_intake: TalonIntakeSubsystem, lights: PiLights):
        super().__init__(
            SequentialCommandGroup(
                ManualExtendHopperCommand(
                    talon_intake, lights, extend=True
                ).withTimeout(0.3),
                WaitCommand(0.1),
                ManualExtendHopperCommand(
                    talon_intake, lights, extend=False
                ).withTimeout(0.3),
                WaitCommand(0.1),
            )
        )
