from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import Rotation2d
from pathplannerlib.pathfinders import Translation2d
from pathplannerlib.trajectory import PathConstraints
from phoenix6.controls.twinkle_off_animation import math
from phoenix6.swerve.swerve_module import Pose2d

from kraken_bot.subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class FaceTarget(Command):
    def __init__(
        self,
        swerve_subsystem: CommandSwerveDrivetrain,
        target_point: Translation2d,
        constraints: PathConstraints,
    ):
        self.swerve_subsystem = swerve_subsystem
        self.target_point = target_point
        self.constraints = constraints

    def execute(self):
        current_position = self.swerve_subsystem.get_state().pose
        target_point = self.target_point

        # Get theta based on where the robot is. Relevant desmos graph: https://www.desmos.com/calculator/kuwylqmvnu
        angle = math.atan(
            (current_position.y - target_point.y)
            / (current_position.x - target_point.x)
        )

        # arctan returns same for left and right sides of graph
        if (target_point.x - current_position.x) < 0 or not (
            (target_point.x - current_position.x) == 0
            and (target_point.y - current_position.y >= 0)
        ):
            # after pi/2, values start at -pi/2, so adding pi makes it positive
            angle += math.pi

        angle_rotation = Rotation2d(angle)
        target_pose = Pose2d(current_position.x, current_position.y, angle_rotation)
        return AutoBuilder.pathfindToPose(target_pose, self.constraints).execute()
