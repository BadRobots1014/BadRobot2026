import commands2

from commands.extend_hopper import ExtendHopperCommand
from subsystems.hopper import HopperSubsystem
from subsystems.pilights import PiLights

WAIT_TIME = 0.1
TIMEOUT = 0.2
JIGGLE_VOLTAGE = 3
JIGGLE_DISTANCE = 10


class JiggleCommand(commands2.RepeatCommand):
    def __init__(
        self,
        hopper: HopperSubsystem,
        lights: PiLights,
    ):
        super().__init__(
            ExtendHopperCommand(
                hopper,
                lights,
                extend=True,
                extension_voltage=JIGGLE_VOLTAGE,
                max_distance_limit=JIGGLE_DISTANCE,
            )
            .withTimeout(TIMEOUT)
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
            .andThen(
                ExtendHopperCommand(
                    hopper,
                    lights,
                    extend=True,
                    extension_voltage=JIGGLE_VOLTAGE,
                    max_distance_limit=JIGGLE_DISTANCE,
                ).withTimeout(TIMEOUT + 0.01)
            )
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
        )
