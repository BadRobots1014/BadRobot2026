from math import cos, sin, tan
from wpimath.controller import PIDController

import commands2
import numpy as np

from phoenix6 import swerve
import kraken_container

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


def calculate(limelight: LimelightSubsystem, swerve_subsystem: CommandSwerveDrivetrain) -> float:
    results = limelight.get_results()
    if not results:
        return 0.0
    positions: list[tuple[float, float]] = []
    for result in results.detectorResults:
        angle_to_goal = (result.target_y_degrees + limelight_angle) * radians
        if angle_to_goal == 0:
            return 0.0
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
    x = np.array(biggest_group[2])[:, 0]
    y = np.array(biggest_group[2])[:, 1]

    robot_pose = swerve_subsystem.get_state().pose

    group_center_robot_relative = (np.mean(x, axis=0) / 39.37, np.mean(y, axis=0) / 39.37)

    robot_relative_angle = (np.atan2(group_center_robot_relative[0], group_center_robot_relative[1]) * 180) / np.pi
    field_relative_angle = robot_relative_angle + robot_pose.rotation().degrees()

    return field_relative_angle

class AutoGatherRotate(commands2.PIDCommand):
    def __init__(self, limelight: LimelightSubsystem, drive_train: CommandSwerveDrivetrain):
        self.pid = PIDController(1, 0, 0)
        self.pid.setTolerance(10)
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(kraken_container.DRIVE_DEADBAND)
            .with_rotational_deadband(kraken_container.ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        super().__init__(
            self.pid,
            lambda: drive_train.get_state().pose.rotation().degrees(),
            calculate(limelight, drive_train),
            lambda output: drive_train.set_control(self._drive.with_rotational_rate(output))
            [limelight, drive_train]
        )
        self.addRequirements(limelight, drive_train)

        def isFinished(self) -> bool:
            return self.pid.atSetpoint()