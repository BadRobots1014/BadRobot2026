import commands2

from commands.extend_hopper import ExtendHopperCommand
from subsystems.intake import IntakeSubsystem
from subsystems.pilights import PiLights

WAIT_TIME = 0.1
TIMEOUT = 0.2
JIGGLE_VOLTAGE = 3
JIGGLE_DISTANCE = 10


class JiggleCommand(commands2.RepeatCommand):
    def __init__(self, intake: IntakeSubsystem, lights: PiLights):
        super().__init__(
            ExtendHopperCommand(
                intake,
                lights,
                extend=True,
                positive_voltage=JIGGLE_VOLTAGE,
                positive_distance_limit=JIGGLE_DISTANCE,
            )
            .withTimeout(TIMEOUT)
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
            .andThen(
                ExtendHopperCommand(
                    intake,
                    lights,
                    extend=True,
                    positive_voltage=JIGGLE_VOLTAGE,
                    positive_distance_limit=JIGGLE_DISTANCE,
                ).withTimeout(TIMEOUT + 0.01)
            )
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
        )
