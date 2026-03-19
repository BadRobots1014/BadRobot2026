import math

import commands2
from phoenix6 import swerve
import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d

import kraken_container  # import file instead of class for constants
from subsystems.shooter import ShooterSubsystem
from subsystems import pilights
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class Strafe(commands2.Command):
    # pass in parent subsystem
    def __init__(
        self,
        swerve_subsystem: CommandSwerveDrivetrain,
        shooter: ShooterSubsystem,
        lights: pilights.PiLights,
        target_point: Translation2d,
        clockwise: bool,
        max_angular_rate: float,
        drive_pid: PIDController,
        rotate_pid: PIDController,
    ):
        super().__init__()
        self.addRequirements(swerve_subsystem)
        # make sure to add requirements to parent subsystem here
        self.swerve_subsystem = swerve_subsystem
        self.shooter_subsystem = shooter
        self.clockwise = clockwise
        self.target_point = target_point
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(kraken_container.DRIVE_DEADBAND)
            .with_rotational_deadband(kraken_container.ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self.max_angular_rate = max_angular_rate

        self.rotate_pid = rotate_pid
        self.drive_pid = drive_pid

        wpilib.SmartDashboard.putData("Strafe rotate pid", self.rotate_pid)
        wpilib.SmartDashboard.putData("Strafe radical pid", self.drive_pid)

        self.lights = lights

    # runs every scheduled tick (think of it as a while true)
    def execute(self) -> None:
        self.lights.set_state(pilights.LEDState.RADIUS)

        # gets current bot pos
        bot_pos = self.swerve_subsystem.get_state().pose

        x_dist = self.target_point.x - bot_pos.x
        y_dist = self.target_point.y - bot_pos.y

        theta = math.atan2(y_dist, x_dist)

        strafe_speed = kraken_container.MAX_SPEED / 3

        r_dist = math.hypot(x_dist, y_dist)
        radius = self.shooter_subsystem.set_radius_pair(r_dist)
        r_output = self.drive_pid.calculate(r_dist, radius)

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
            self.rotate_pid.calculate(
                self.swerve_subsystem.get_state().pose.rotation().radians(), theta
            )
            * self.max_angular_rate
        )

        self.swerve_subsystem.set_control(
            self._drive.with_velocity_x(vx)
            .with_velocity_y(vy)
            .with_rotational_rate(rotational_rate)
        )
