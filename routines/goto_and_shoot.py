from commands2 import SequentialCommandGroup

from subsystems.shooter import ShooterSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class GotoAndShoot(SequentialCommandGroup):
    def __init__(self, shooter: ShooterSubsystem, drive: CommandSwerveDrivetrain):
        super().__init__()
