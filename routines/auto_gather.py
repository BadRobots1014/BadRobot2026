from commands2 import SequentialCommandGroup, RepeatCommand

from subsystems.limelight import LimelightSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

from commands.auto_rotate import AutoGatherRotate
from commands.drive_forward import DriveForward

class AutoGather(SequentialCommandGroup):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, limelight: LimelightSubsystem):
        super().__init__(
            AutoGatherRotate(limelight, drivetrain),
            DriveForward(drivetrain)
        )