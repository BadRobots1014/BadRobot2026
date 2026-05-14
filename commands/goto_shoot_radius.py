from collections.abc import Callable
import math

from commands2 import Command
from phoenix6 import swerve
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d

import kraken_container
from subsystems.shooter import ShooterSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

TRANSLATION_THRESHOLD = 0.2  # distance in meters away from r
ROTATION_THRESHOLD = 0.05  # .1  # radians away from target_theta


class GotoShootRadius(Command):
    def __init__(
        self,
        swerve_subsystem: CommandSwerveDrivetrain,
        shooter: ShooterSubsystem,
        target_point: Callable[[], Translation2d],
        blue_alliance: bool,
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
        self.blue_alliance = blue_alliance
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

        # DO NOT ADD SHOOTER TO THIS, WE DON'T WANT THIS INTERRUPTING SHOOT
        self.addRequirements(self.swerve_subsystem)

        super().__init__()

    def execute(self) -> None:
        bot_pos = self.swerve_subsystem.get_state().pose
        x_dist = self.target_point().x - bot_pos.x
        y_dist = self.target_point().y - bot_pos.y

        self.target_theta = math.atan2(y_dist, x_dist)
        self.current_theta = self.swerve_subsystem.get_state().pose.rotation().radians()

        self.r_dist = math.hypot(x_dist, y_dist)

        temp_theta = (
            self.target_theta + math.pi if self.blue_alliance else self.target_theta
        )
        ignore_pairs = []

        if abs(temp_theta) < 1.85:
            ignore_pairs = [0, 1, 2, 3]
        # NO SHOOTING - 1.85 to 1.85

        elif abs(temp_theta) < 2:
            ignore_pairs = [0, 1, 3]
        # ALLOW 3.4 - 1.85 to 2 -1.85 to -2

        elif abs(temp_theta) < 2.25:
            ignore_pairs = [0, 3]
        # ALLOW 2.8 3.4 - 2 to 2.25 -2 -2.25

        elif temp_theta < 2.9 and temp_theta > -2.75:
            ignore_pairs = []
        # ALLOW 2.235 2.8 3.4 4.1 - 2.25 to 2.9 -2.25 to -2.75

        else:
            ignore_pairs = [2, 3]
        # NO 3.4 4.1 - 2.9 to -2.75

        ignore_pairs = []

        pair = self.shooter.set_radius_pair(self.r_dist, ignore_pairs)
        if pair is None:
            self.no_pair = True
            return
        self.no_pair = False

        self.radius = pair[0]

        r_output = self.drive_pid.calculate(self.radius, self.r_dist)

        ux = x_dist / self.r_dist
        uy = y_dist / self.r_dist

        vx_radical = r_output * ux
        vy_radical = r_output * uy

        rotational_rate = (
            self.rotate_pid.calculate(self.current_theta, self.target_theta)
            * kraken_container.MAX_ANGULAR_SPEED
        )

        self.swerve_subsystem.set_control(
            self._drive.with_velocity_x(vx_radical)
            .with_velocity_y(vy_radical)
            .with_rotational_rate(rotational_rate)
        )

    def isFinished(self) -> bool:
        if self.no_pair or (
            abs(self.r_dist - self.radius) < TRANSLATION_THRESHOLD
            and abs(self.current_theta - self.target_theta) < ROTATION_THRESHOLD
        ):
            return True
        else:
            return False

    def end(self, interrupted: bool) -> None:
        self.swerve_subsystem.set_control(
            self._drive.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        )
