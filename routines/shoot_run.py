from commands2 import ParallelCommandGroup, SequentialCommandGroup
from wpimath.geometry import Translation2d

from commands.goto_commands import goto_shoot_pos
from commands.shoot import ShootCommand
from commands.shoot_kicker import ShootKickerCommand
from subsystems.shooter import ShooterSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain
import subsystems.pilights as pilights


class ShootRunRoutine(ParallelCommandGroup):
    """
    Drive to shooting position and shoot into hub
    """

    def __init__(
        self,
        shooter: ShooterSubsystem,
        target_point: Translation2d,
        swerve_subsystem: CommandSwerveDrivetrain,
        max_speed: float,
        max_angular_speed_rads: float,
        max_acceleration: float,
        max_angular_acceleration_rads: float,
    ):
        super().__init__()
        self.GotoShoot = goto_shoot_pos(
            target_point,
            swerve_subsystem,
            max_speed,
            max_angular_speed_rads,
            max_acceleration,
            max_angular_acceleration_rads,
        )
        self.addCommands(
            ShootCommand(shooter),
            SequentialCommandGroup(
                self.GotoShoot,
                ShootKickerCommand(shooter),
            ),
        )
