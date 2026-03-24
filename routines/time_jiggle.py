from commands2 import RepeatCommand, SequentialCommandGroup, WaitCommand

from commands.manual_extend_hopper import ManualExtendHopperCommand
from subsystems.hopper import HopperSubsystem
from subsystems.pilights import PiLights


class TimeJiggleRoutine(RepeatCommand):
    def __init__(self, hopper: HopperSubsystem, lights: PiLights):
        super().__init__(
            SequentialCommandGroup(
                ManualExtendHopperCommand(hopper, lights, extend=True).withTimeout(0.3),
                WaitCommand(0.1),
                ManualExtendHopperCommand(hopper, lights, extend=False).withTimeout(
                    0.3
                ),
                WaitCommand(0.1),
            )
        )
