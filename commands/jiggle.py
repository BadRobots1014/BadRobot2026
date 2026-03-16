import commands2

from commands.extend_hopper import ExtendHopperCommand
from subsystems.intake import IntakeSubsystem
from subsystems.talonFXIntake import TalonIntakeSubsystem

WAIT_TIME = 0.1
TIMEOUT = 0.2


class JiggleCommand(commands2.RepeatCommand):
    def __init__(self, intake: IntakeSubsystem | TalonIntakeSubsystem):
        super().__init__(
            ExtendHopperCommand(intake, extend=True)
            .withTimeout(TIMEOUT)
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
            .andThen(ExtendHopperCommand(intake, extend=True).withTimeout(TIMEOUT))
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
        )
        self.addRequirements(intake)
