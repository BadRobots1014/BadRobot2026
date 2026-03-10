from commands2 import ParallelCommandGroup

from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem


class ShootInPlace(ParallelCommandGroup):
    def __init__(self, intake: IntakeSubsystem, shoot: ShooterSubsystem):
        self.addRequirements(intake, shoot)
        super().__init__()
        self.addCommands()
