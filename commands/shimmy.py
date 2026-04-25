import math

from commands2 import Command
from phoenix6 import swerve
import wpilib
from wpilib import Timer
from wpimath._controls._controls.controller import PIDController

import kraken_container
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

SHIMMY_P = 1
SHIMMY_I = 0
SHIMMY_D = 0


class Shimmy(Command):
    def __init__(self, drive: CommandSwerveDrivetrain):
        super().__init__()
        self.addRequirements(drive)
        self.drive = drive
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(kraken_container.DRIVE_DEADBAND)
            .with_rotational_deadband(kraken_container.ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self.angle = 0
        self.start_time = 0
        self.shimmy_pid = PIDController(SHIMMY_P, SHIMMY_I, SHIMMY_D)
        wpilib.SmartDashboard.putData(self.shimmy_pid)

    def initialize(self) -> None:
        self.angle = self.drive.get_state().pose.rotation().radians()
        self.start_time = Timer.getFPGATimestamp()
        self.drive.set_control(
            self._drive.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(2)
        )

    def execute(self) -> None:
        set_point = self.angle - math.sin(
            (Timer.getFPGATimestamp() - self.start_time) * 16
        )
        vr = self.shimmy_pid.calculate(
            self.drive.get_state().pose.rotation().radians(), set_point
        )

        self.drive.set_control(
            self._drive.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(vr)
        )
