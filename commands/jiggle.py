import commands2

from commands.extend_hopper import ExtendHopperCommand
from subsystems.intake import IntakeSubsystem


class JiggleCommand(commands2.RepeatCommand):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__(ExtendHopperCommand)
        self.addRequirements(intake)
