from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.extend_hopper import ExtendHopper
from commands.run_intake import RunIntake
from commands.run_seesaw import RunSeesaw
from subsystems.intake import Intake
from subsystems.seesaw import Seesaw


class Reload(SequentialCommandGroup):
    def __init__(self, intake: Intake, seesaw: Seesaw):
        super().__init__()
        self.addCommands(
            RunSeesaw(seesaw, True),
            ExtendHopper(intake, False),
            RunSeesaw(seesaw, False),
        )
