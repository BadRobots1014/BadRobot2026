from commands2 import Command
from commands2.button import CommandGenericHID
from pathplannerlib.controller import PIDController
from pathplannerlib.pathfinders import Translation2d
from phoenix6.controls.twinkle_off_animation import math
from phoenix6.swerve.requests import FieldCentric

from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

PID_P = 1
PID_I = 0
PID_D = 0


def FaceTargetCommand(
    swerve_subsystem: CommandSwerveDrivetrain,
    target_point: Translation2d,
    field: FieldCentric,
    joystick: CommandGenericHID,
    max_speed: float,
    max_angular_rate: float,
    left_y_axis: int,
    left_x_axis: int,
) -> Command:
    rotate_pid = PIDController(PID_P, PID_I, PID_D)
    rotate_pid.enableContinuousInput(0, 2 * math.pi)
    return swerve_subsystem.apply_request(
        lambda: (
            field.with_velocity_x(-joystick.getRawAxis(left_y_axis) * max_speed)
            .with_velocity_y(-joystick.getRawAxis(left_x_axis) * max_speed)
            .with_rotational_rate(
                rotate_pid.calculate(
                    swerve_subsystem.get_state().pose.rotation().radians(),
                    # Get theta from target point and current position
                    math.atan2(
                        (target_point.y - swerve_subsystem.get_state().pose.y),
                        (target_point.x - swerve_subsystem.get_state().pose.x),
                    ),
                )
                * max_angular_rate
            )
        )
    ).until(rotate_pid.atSetpoint)
