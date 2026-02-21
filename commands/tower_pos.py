import commands2
from pathplannerlib.auto import AutoBuilder
import math

from pathplannerlib.auto import PathConstraints
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

# TODO: arbitrary
DESIRED_RADIUS_METER = 5

def go_to_shoot_pos(
    target_point: Translation2d,
    swerve_subsystem: CommandSwerveDrivetrain,
    max_speed: float,
    max_angular_speed_rads: float,
    max_acceleration: float,
    max_angular_acceleration_rads: float,
):
    path_constraints = PathConstraints(
        max_speed,
        max_acceleration,
        max_angular_speed_rads,
        max_angular_acceleration_rads,
    )
    bot_pos = swerve_subsystem.get_state().pose.position
    target_point = target_point
    distance = math.hypot(
        bot_pos.x - target_point.x, bot_pos.y - target_point.y
    )
    ratio = distance / DESIRED_RADIUS_METER
    vector = (bot_pos.x - target_point.x, bot_pos.y - target_point.y)
    vector[0] *= ratio
    vector[1] *= ratio
    goal_rotation = math.atan2(
        (target_point.y - swerve_subsystem.get_state().pose.y),
        (target_point.x - swerve_subsystem.get_state().pose.x),
    )
    goal_pos = Pose2d(
        vector[0],
        vector[1],
        Rotation2d.fromRotations(goal_rotation / (2 * math.pi)),
    )
    command = AutoBuilder.pathfindToPose(goal_pos, path_constraints)

    return command
