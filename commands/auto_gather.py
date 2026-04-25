from math import cos, sin, tan

import commands2
import numpy as np

from subsystems.limelight import LimelightSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

limelight_angle = -10.0
limelight_height = 0  # TODO (in)
fuel_height = 0  # TODO (inches)

radians = 3.14159 / 180

group_limit = 12
intake_length = 24

box_size = 5

cam_x_to_bot = 0
cam_y_to_bot = 0


# limelight_fov = 62.5 * (3.14159 / 180)
# limelight_width_px = 1280


def calculate(limelight: LimelightSubsystem, swerve_subsystem: CommandSwerveDrivetrain):
    results = limelight.get_results()
    if not results:
        return
    positions: list[tuple[float, float]] = []
    for result in results.detectorResults:
        angle_to_goal = (result.target_y_degrees + limelight_angle) * radians
        if angle_to_goal == 0:
            return  # TODO redo calc at next tick
        radius = (fuel_height - limelight_height) / tan(angle_to_goal)
        x = radius * cos(result.target_x_degrees * radians)
        y = radius * sin(result.target_x_degrees * radians)
        positions.append((x, y))

    groups: list[
        tuple[tuple[float, float], tuple[float, float], list[tuple[float, float]]]
    ] = []

    for position in positions:
        added_to_group = False
        for i, group in enumerate(groups):
            if (
                group[0][0] <= position[0] < group[1][0]
                and group[0][1] <= position[1] < group[1][1]
            ):
                added_to_group = True
                group[2].append((position[0], position[1]))

                if position[0] - 12 < group[0][0]:
                    groups[i] = (
                        (position[0] - 12, group[0][1]),
                        group[1],
                        group[2],
                    )
                elif position[0] + 12 > group[1][0]:
                    groups[i] = (
                        group[0],
                        (position[1] + 12, group[1][1]),
                        group[2],
                    )

                if position[1] - 12 < group[0][0]:
                    groups[i] = (
                        (group[0][0], position[1] - 12),
                        group[1],
                        group[2],
                    )
                elif position[1] + 12 > group[1][0]:
                    groups[i] = (
                        group[0],
                        (group[1][0], position[1] + 12),
                        group[2],
                    )

        if not added_to_group:
            groups.append(
                (
                    (
                        position[0] - group_limit,
                        position[1] - group_limit,
                    ),  # Make group limit large enough so you only have to check with the center and not another box
                    (position[0] + group_limit, position[1] + group_limit),
                    [(position[0], position[1])],
                )
            )

    biggest_group = max(groups, key=len)
    group_center_bot_relative = (
        (biggest_group[1][0] - biggest_group[0][0]) * 39.37
        - cam_x_to_bot,  # to meters from inches
        (biggest_group[1][1] - biggest_group[0][1]) * 39.37 - cam_y_to_bot,
    )
    bot_pos = swerve_subsystem.get_state().pose
    rotation = swerve_subsystem.get_state().pose.rotation().radians()
    # Do not really trust this math. Do testing to see what goes wrong with values
    group_center = (
        bot_pos.x
        + (
            group_center_bot_relative[0] * cos(rotation)
            - group_center_bot_relative[1] * sin(rotation)
        ),
        bot_pos.y
        + (
            group_center_bot_relative[0] * sin(rotation)
            + group_center_bot_relative[1] * cos(rotation)
        ),
    )
    # TODO move to pose logic


class ExampleCommand(commands2.Command):
    # pass in parent subsystem
    def __init__(self, limelight: LimelightSubsystem):
        super().__init__()
        self.addRequirements(limelight)
        # make sure to add requirements to parent subsystem here

    # runs every scheduled tick (think of it as a while true)
    def execute(self) -> None:
        pass

    # boolean condition to check if the command is finished (needed for running commands in series)
    def isFinished(self) -> bool:
        return False

    # code that runs after the command is finished
    def end(self, interrupted: bool) -> None:
        pass
