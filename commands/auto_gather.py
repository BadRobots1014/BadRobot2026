from math import cos, sin, tan

import commands2
from numpy.polynomial import Polynomial

from subsystems.limelight import LimelightSubsystem
import numpy as np
from sklearn.linear_model import LinearRegression

limelight_angle = -10.0
limelight_height = 0  # TODO (in)
fuel_height = 0  # TODO (inches)

radians = 3.14159 / 180

group_limit = 12
intake_length = 24

box_size = 5


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

    groups: list[tuple[tuple[float, float], tuple[float, float], list[tuple[float, float]]]] = []

    for position in positions:
        if not groups:
            groups.append(
                (
                    (position[0] - group_limit, position[1] - group_limit), # Make group limit large enough so you only have to check with the center and not another box
                    (position[0] + group_limit, position[1] + group_limit),
                    [(position[0], position[1])],
                )
            )
            continue

        for i, group in enumerate(groups):
            if group[0][0] <= position[0] <group[1][0] and group[0][1] <= position[1] < group[1][1]:
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

    np_groups = np.array(groups)
    best_group = ()

    for group in np_groups:
        x = group[2][:, 0]
        y = group[2][:, 1]

        model = LinearRegression()
        model.fit([[i] for i in x], y)

        distances = np.abs(model.coef_ * x - y + model.intercept_) / np.sqrt(model.coef_ ** 2 + 1)

        num_collected = len(np.where(distances < intake_length)[0])

        if num_collected > best_group[0]:
            best_group = (group, model)

    max_point_x = max(best_group[0][2], key=lambda point: point[0])

    best_model = best_group[1]
    goto_position = (max_point_x, best_model.predict([max_point_x]))

    return goto_position


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
