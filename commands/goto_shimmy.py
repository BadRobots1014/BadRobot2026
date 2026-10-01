from collections.abc import Callable
import math

from commands2.conditionalcommand import ConditionalCommand
from wpimath.geometry import Translation2d

from commands.goto_shoot_radius import TRANSLATION_THRESHOLD, GotoShootRadius
from commands.shimmy import Shimmy
from drive_wrapper import DriveWrapper
from subsystems.shooter import ShooterSubsystem


class GoToShimmy(ConditionalCommand):
    def __init__(self, drive_wrapper: DriveWrapper, shooter: ShooterSubsystem, target_point: Callable[[], Translation2d], blue_alliance: bool):
        # Shooter does not need an add requirements.
        self.addRequirements(drive_wrapper.drivetrain)
        self.drive_wrapper = drive_wrapper
        self.target_point = target_point
        self.blue_alliance = blue_alliance
        self.shooter = shooter
        super().__init__(Shimmy(drive_wrapper), GotoShootRadius(drive_wrapper, shooter, target_point, blue_alliance), self.check)

    def check(self) -> bool:
        bot_pos = self.drive_wrapper.drivetrain.get_state().pose
        x_dist = self.target_point().x - bot_pos.x
        y_dist = self.target_point().y - bot_pos.y

        target_theta = math.atan2(y_dist, x_dist)

        r_dist = math.hypot(x_dist, y_dist)

        temp_theta = (
            target_theta + math.pi if self.blue_alliance else target_theta
        )
        ignore_pairs = []

        if abs(temp_theta) < 1.85:
            ignore_pairs = [0, 1, 2, 3]
        # NO SHOOTING - 1.85 to 1.85

        elif abs(temp_theta) < 2:
            ignore_pairs = [0, 1, 3]
        # ALLOW 3.4 - 1.85 to 2 -1.85 to -2

        elif abs(temp_theta) < 2.25:
            ignore_pairs = [0, 3]
        # ALLOW 2.8 3.4 - 2 to 2.25 -2 -2.25

        elif 2.9 > temp_theta > -2.75:
            ignore_pairs = []
        # ALLOW 2.235 2.8 3.4 4.1 - 2.25 to 2.9 -2.25 to -2.75

        else:
            ignore_pairs = [2, 3]
        # NO 3.4 4.1 - 2.9 to -2.75

        pair = self.shooter.set_radius_pair(r_dist, ignore_pairs)
        radius = pair[0]
        return abs(r_dist - radius) < TRANSLATION_THRESHOLD
