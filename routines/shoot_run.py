from commands2 import ParallelCommandGroup, SequentialCommandGroup
from wpimath.geometry import Translation2d

from commands.shoot import Shoot
from commands.shoot_kicker import Shoot_Kicker
from commands.run_seesaw import RunSeesaw
from commands.goto_commands import goto_shoot_pos
from subsystems.shooter import Shooter
from subsystems.seesaw import Seesaw
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class ShootRunRoutine(ParallelCommandGroup):
    def __init__(
        self,
        shooter: Shooter,
        seesaw: Seesaw,
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
            Shoot(shooter),
            SequentialCommandGroup(
                ParallelCommandGroup(RunSeesaw(seesaw, False), self.GotoShoot),
                Shoot_Kicker(shooter),
            ),
        )
