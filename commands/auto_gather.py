from math import cos, sin, tan

import commands2
from numpy.polynomial import Polynomial

from subsystems.limelight import LimelightSubsystem

limelight_angle = -10.0
limelight_height = 0  # TODO (in)
fuel_height = 0  # TODO (inches)

radians = 3.14159 / 180

group_limit = 12
intake_length = 24


# limelight_fov = 62.5 * (3.14159 / 180)
# limelight_width_px = 1280


def calculate(limelight: LimelightSubsystem):
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
        for i, group in enumerate(groups):
            group_x_limits = group[0]
            group_y_limits = group[1]
            if (
                position[0] > group_x_limits[0]
                and position[0] < group_x_limits[1]
                and position[1] > group_y_limits[0]
                and position[1] < group_y_limits[1]
            ):
                group[2].append((position[0], position[1]))
                if position[0] - 12 < group_x_limits[0]:
                    groups[i] = (
                        (position[0] - 12, group_x_limits[1]),
                        groups[i][1],
                        groups[i][2],
                    )
                elif position[0] + 12 > group_x_limits[1]:
                    groups[i] = (
                        (group_x_limits[0], position[0] + 12),
                        groups[i][1],
                        groups[i][2],
                    )
                elif position[1] - 12 < group_y_limits[0]:
                    groups[i] = (
                        groups[i][0],
                        (position[1] - 12, group_y_limits[1]),
                        groups[i][2],
                    )
                elif position[1] + 12 > group_y_limits[1]:
                    groups[i] = (
                        groups[i][0],
                        (group_y_limits[0], position[1] + 12),
                        groups[i][2],
                    )
                break
        groups.append(
            (
                (position[0] - group_limit, position[0] + group_limit),
                (position[1] - group_limit, position[1] + group_limit),
                [(position[0], position[1])],
            )
        )

    best_group = (0, ())
    for group in groups:
        m, b = Polynomial.fit(group[2][0], group[2][1], 1).convert().coef
        points = [(group[2][0][i], group[2][1][i]) for i in range(len(group[2][0]))]
        balls_collected = 0
        for point in points:
            if abs(point[1] - (m * point[0] + b)) > intake_length:
                balls_collected += 1
        if balls_collected > best_group[0]:
            best_group = (balls_collected, group)


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
