from collections.abc import Callable
import math

from commands2 import Command
from phoenix6 import swerve
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d

import kraken_container
from subsystems.shooter import ShooterSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

THRESHOLD = 0.15  # distance in meters away from r


class GotoShootRadius(Command):
    def __init__(
        self,
        swerve_subsystem: CommandSwerveDrivetrain,
        shooter: ShooterSubsystem,
        target_point: Callable[[], Translation2d],
        drive_pid: PIDController,
        rotate_pid: PIDController,
    ) -> None:
        """
        Attempt to maintain THRESHOLD distance from `target_point`

        :param target_point: WPILib position (blue centered) of desired location.
        """
        self.swerve_subsystem = swerve_subsystem
        self.shooter = shooter
        self.target_point = target_point
        self.drive_pid = drive_pid
        self.rotate_pid = rotate_pid

        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(kraken_container.DRIVE_DEADBAND)
            .with_rotational_deadband(kraken_container.ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
            .with_forward_perspective(
                swerve.requests.ForwardPerspectiveValue.BLUE_ALLIANCE
            )
        )

        self.r_dist = 0
        self.radius = 3

        # DO NOT ADD SHOOTER TO THIS, WE DON'T WANT THIS INTERRUPTING SHOOT
        self.addRequirements(self.swerve_subsystem)

        super().__init__()

    def execute(self) -> None:
        bot_pos = self.swerve_subsystem.get_state().pose
        x_dist = self.target_point().x - bot_pos.x
        y_dist = self.target_point().y - bot_pos.y

        theta = math.atan2(y_dist, x_dist)

        self.r_dist = math.hypot(x_dist, y_dist)

        self.radius = self.shooter.set_radius_pair(self.r_dist)
        r_output = self.drive_pid.calculate(self.radius, self.r_dist)

        ux = x_dist / self.r_dist
        uy = y_dist / self.r_dist

        vx_radical = r_output * ux
        vy_radical = r_output * uy

        rotational_rate = (
            self.rotate_pid.calculate(
                self.swerve_subsystem.get_state().pose.rotation().radians(), theta
            )
            * kraken_container.MAX_ANGULAR_SPEED
        )

        self.swerve_subsystem.set_control(
            self._drive.with_velocity_x(vx_radical)
            .with_velocity_y(vy_radical)
            .with_rotational_rate(rotational_rate)
        )

    def isFinished(self) -> bool:
        # print("Threshold "+str(self.r_dist) + " " + str(self.radius + THRESHOLD))
        if abs(self.r_dist - self.radius) < THRESHOLD:
            return True
        else:
            return False

    def end(self, interrupted: bool) -> None:
        self.swerve_subsystem.set_control(
            self._drive.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        )
