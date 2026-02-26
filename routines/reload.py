from commands2 import SequentialCommandGroup

from commands.extend_hopper import ExtendHopperCommand
from commands.run_seesaw import RunSeesawCommand

from subsystems.intake import IntakeSubsystem
from subsystems.seesaw import SeesawSubsystem


class ReloadRoutine(SequentialCommandGroup):
    """
    Reload the by setting the seesaw to intake, retracting the hopper, and resetting the seesaw to shoot.
    """

    def __init__(self, intake: IntakeSubsystem, seesaw: SeesawSubsystem):
        super().__init__()
        self.addCommands(
            RunSeesawCommand(seesaw, True),
            ExtendHopperCommand(intake, False),
            RunSeesawCommand(seesaw, False),
        )
