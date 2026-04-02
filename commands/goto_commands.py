import math

from commands2 import Command
from pathplannerlib.auto import AutoBuilder, PathConstraints
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

# TODO: arbitrary
DESIRED_RADIUS_METER = 5


def goto_shoot_pos(
    target_point: Translation2d,
    swerve_subsystem: CommandSwerveDrivetrain,
    max_speed: float,
    max_angular_speed_rads: float,
    max_acceleration: float,
    max_angular_acceleration_rads: float,
) -> Command:

    path_constraints = PathConstraints(
        max_speed,
        max_acceleration,
        max_angular_speed_rads,
        max_angular_acceleration_rads,
    )
    # gets current bot pos
    bot_pos = swerve_subsystem.get_state().pose
    # gets distance from current pos to tower pos
    distance = math.hypot(bot_pos.x - target_point.x, bot_pos.y - target_point.y)
    # get scalar for radius ratio
    ratio = distance / DESIRED_RADIUS_METER
    # distance vector & scale vector by ratio
    vector = (
        (bot_pos.x - target_point.x) * ratio,
        (bot_pos.y - target_point.y) * ratio,
    )
    # calculate goal rotation
    goal_rotation = math.atan2(
        (target_point.y - bot_pos.y),
        (target_point.x - bot_pos.x),
    )
    goal_pos = Pose2d(
        vector[0],
        vector[1],
        Rotation2d.fromRotations(goal_rotation / (2 * math.pi)),
    )
    command = AutoBuilder.pathfindToPose(goal_pos, path_constraints)

    return command
