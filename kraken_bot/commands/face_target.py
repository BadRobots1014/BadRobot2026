from commands2 import Command
from commands2.button import CommandXboxController
from pathplannerlib.controller import PIDController
from pathplannerlib.pathfinders import Translation2d
from phoenix6.controls.twinkle_off_animation import math
from phoenix6.swerve.requests import FieldCentric

from kraken_bot.subsystems.swerve_drivetrain import CommandSwerveDrivetrain


def FaceTarget(
    swerve_subsystem: CommandSwerveDrivetrain,
    target_point: Translation2d,
    field: FieldCentric,
    joystick: CommandXboxController,
    max_speed: float,
    max_angular_rate: float,
) -> Command:
    rotate_pid = PIDController(1, 0, 0)
    rotate_pid.enableContinuousInput(0, 2 * math.pi)
    return swerve_subsystem.apply_request(
        lambda: (
            field.with_velocity_x(-joystick.getLeftY() * max_speed)
            .with_velocity_y(-joystick.getLeftX() * max_speed)
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
    ).until(lambda: rotate_pid.atSetpoint())
