from collections.abc import Callable
import math

from commands2 import Command
from pathplannerlib.auto import AutoBuilder, PathConstraints, PathPlannerPath
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystems import pilights
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

# TODO: arbitrary
DESIRED_RADIUS_METER = 5

MIDPOINT_Y = 4
BLUE_TRENCH_X = 4.5
RED_TRENCH_X = 11.9


def goto_shoot_pos(
    target_point: Translation2d,
    swerve_subsystem: CommandSwerveDrivetrain,
    max_speed: float,
    max_angular_speed_rads: float,
    max_acceleration: float,
    max_angular_acceleration_rads: float,
    lights: pilights.PiLights,
) -> Command:
    lights.set_state(pilights.LEDState.AUTO)

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


def go_through_trench(
    is_blue: bool,
    robot_pose_supplier: Callable[[], Pose2d],
    constraints: PathConstraints,
) -> Command:
    trench_x = BLUE_TRENCH_X if is_blue else RED_TRENCH_X
    pose_left_of_midpoint = (robot_pose_supplier().Y() < MIDPOINT_Y) ^ is_blue
    pose_in_neutral_zone = (robot_pose_supplier().X() < trench_x) ^ is_blue

    if pose_in_neutral_zone:
        path = PathPlannerPath.fromPathFile("Trench Left Out")
    else:
        path = PathPlannerPath.fromPathFile("Trench Left In")

    if not pose_left_of_midpoint:
        path = path.mirrorPath()

    return AutoBuilder.pathfindThenFollowPath(path, constraints)
