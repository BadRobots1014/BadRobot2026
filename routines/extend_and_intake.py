from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.extend_hopper import ExtendHopper
from commands.run_intake import RunIntake
from commands.run_seesaw import RunSeesaw
from subsystems.intake import Intake
from subsystems.seesaw import Seesaw


class ExtendAndIntake(ParallelCommandGroup):
    def __init__(self, intake: Intake, seesaw: Seesaw, dump: bool):
        super().__init__()
        self.addCommands(
            SequentialCommandGroup(ExtendHopper(intake, True), RunIntake(intake, dump)),
            RunSeesaw(seesaw, dump),
        )
