from commands.manual_extension_command import ManualExtensionCommand
import commands2

from subsystems.intake import IntakeSubsystem

WAIT_TIME = 0.1
TIMEOUT = 0.2


class JiggleCommand(commands2.RepeatCommand):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__(
            ManualExtensionCommand(intake, extend=True)
            .withTimeout(TIMEOUT)
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
            .andThen(ManualExtensionCommand(intake, extend=True).withTimeout(TIMEOUT))
            .andThen(commands2.waitcommand.WaitCommand(WAIT_TIME))
        )
        self.addRequirements(intake)
