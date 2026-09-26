from collections.abc import Callable
import math

import commands2
from phoenix6 import swerve
import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d
from wpimath.units import rotationsToRadians

import drive_wrapper
import kraken_container  # import file instead of class for constants
from drive_wrapper import DriveWrapper
from subsystems.shooter import ShooterSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

class Strafe(commands2.Command):
    # pass in parent subsystem
    def __init__(
        self,
        swerve_subsystem: DriveWrapper,
        shooter: ShooterSubsystem,
        target_point: Callable[[], Translation2d],
        clockwise: bool,
    ):
        super().__init__()

        self.drive_wrapper = swerve_subsystem
        self.addRequirements(self.drive_wrapper.drivetrain)

        self.swerve_subsystem = swerve_subsystem
        self.shooter_subsystem = shooter
        self.clockwise = clockwise
        self.target_point = target_point

        wpilib.SmartDashboard.putData("Strafe rotate pid", self.drive_wrapper.rotate_pid)
        wpilib.SmartDashboard.putData("Strafe radical pid", self.drive_wrapper.drive_pid)

    # runs every scheduled tick (think of it as a while true)
    def execute(self) -> None:
        # self.lights.set_state(pilights.LEDState.RADIUS)

        # gets current bot pos
        bot_pos = self.swerve_subsystem.drivetrain.get_state().pose

        x_dist = self.target_point().x - bot_pos.x
        y_dist = self.target_point().y - bot_pos.y

        theta = math.atan2(y_dist, x_dist)

        strafe_speed = drive_wrapper.MAX_SPEED / 3

        r_dist = math.hypot(x_dist, y_dist)
        radius = self.shooter_subsystem.set_radius_pair(r_dist)
        r_output = self.drive_wrapper.drive_pid.calculate(r_dist, radius)

        ux = x_dist / r_dist
        uy = y_dist / r_dist

        vx_radical = r_output * ux
        vy_radical = r_output * uy

        if self.clockwise:
            vx_tangent = strafe_speed * math.sin(theta)
            vy_tangent = -strafe_speed * math.cos(theta)
        else:
            vx_tangent = -strafe_speed * math.sin(theta)
            vy_tangent = strafe_speed * math.cos(theta)

        vx = vx_tangent + vx_radical
        vy = vy_tangent + vy_radical

        rotational_rate = (
            self.drive_wrapper.rotate_pid.calculate(
                self.drive_wrapper.drivetrain.get_state().pose.rotation().radians(), theta
            )
            * self.drive_wrapper.max_angular_speed
        )

        self.drive_wrapper.field_centric_drive(vx, vy, rotational_rate)