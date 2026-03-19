import commands2

from commands.extend_hopper import ExtendHopperCommand
from subsystems.pilights import PiLights
from subsystems.talonFXIntake import TalonIntakeSubsystem

WAIT_TIME = 0.1
TIMEOUT = 0.2
JIGGLE_VOLTAGE = 3
JIGGLE_DISTANCE = 10


class JiggleCommand(commands2.RepeatCommand):
    def __init__(self, intake: TalonIntakeSubsystem, lights: PiLights):
        super().__init__(
            ExtendHopperCommand(
                intake,
                lights,
                extend=True,
                positive_voltage=JIGGLE_VOLTAGE,
                positive_distance_limit=10,
            )
            .withTimeout(TIMEOUT)
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
            .andThen(
                ExtendHopperCommand(
                    intake,
                    lights,
                    extend=True,
                    positive_voltage=JIGGLE_VOLTAGE,
                    positive_distance_limit=10,
                ).withTimeout(TIMEOUT + 0.01)
            )
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
        )
        self.addRequirements(intake)
