from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from commands.run_seesaw import RunSeesawCommand

from subsystems.intake import IntakeSubsystem
from subsystems.seesaw import SeesawSubsystem


class ExtendAndIntakeRoutine(ParallelCommandGroup):
    def __init__(self, intake: IntakeSubsystem, seesaw: SeesawSubsystem, dump: bool):
        super().__init__()
        self.addCommands(
            SequentialCommandGroup(
                ExtendHopperCommand(intake, True), RunIntakeCommand(intake, dump)
            ),
            RunSeesawCommand(seesaw, dump),
        )
