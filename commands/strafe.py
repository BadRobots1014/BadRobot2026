import math

import commands2
from phoenix6 import swerve
from wpimath.geometry import Translation2d

import kraken_container  # import file instead of class for constants
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class Strafe(commands2.Command):
    # pass in parent subsystem
    def __init__(
        self,
        swerve_subsystem: CommandSwerveDrivetrain,
        target_point: Translation2d,
        clockwise: bool,
    ):
        super().__init__()
        self.addRequirements(swerve_subsystem)
        # make sure to add requirements to parent subsystem here
        self.swerve_subsystem = swerve_subsystem
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

    # runs every scheduled tick (think of it as a while true)
    def execute(self) -> None:
        # gets current bot pos
        bot_pos = self.swerve_subsystem.get_state().pose

        theta = math.atan2(
            (self.target_point.y - bot_pos.y),
            (self.target_point.x - bot_pos.x),
        )

        strafe_speed = kraken_container.MAX_SPEED / 5

        print("XVEL: ", (strafe_speed * math.cos(theta)))
        print("YVEL: ", (strafe_speed * math.sin(theta)))
        print("Clockwise: ", self.clockwise)

        # x: strafe_speed * math.cos(theta)
        # y: strafe_speed * math.sin(theta)

        if self.clockwise:
            self.swerve_subsystem.set_control(
                self._drive.with_velocity_x(strafe_speed * math.sin(theta))
                .with_velocity_y(-(strafe_speed * math.cos(theta)))
                .with_rotational_rate(0)
            )
        else:
            self.swerve_subsystem.set_control(
                self._drive.with_velocity_x(-(strafe_speed * math.sin(theta)))
                .with_velocity_y(strafe_speed * math.cos(theta))
                .with_rotational_rate(0)
            )
