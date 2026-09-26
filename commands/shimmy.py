import math

from commands2 import Command
from phoenix6 import swerve
import wpilib
from wpilib import Timer
from wpimath._controls._controls.controller import PIDController

from drive_wrapper import DriveWrapper
import kraken_container
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain

SHIMMY_P = 1
SHIMMY_I = 0
SHIMMY_D = 0


class Shimmy(Command):
    def __init__(self, drive_wrapper: DriveWrapper):
        super().__init__()
        self.addRequirements(drive_wrapper.drivetrain)
        self.drive_wrapper = drive_wrapper
        self.angle = 0
        self.start_time = 0
        self.shimmy_pid = PIDController(SHIMMY_P, SHIMMY_I, SHIMMY_D)
        wpilib.SmartDashboard.putData(self.shimmy_pid)

    def initialize(self) -> None:
        self.angle = self.drive_wrapper.drivetrain.get_state().pose.rotation().radians()
        self.start_time = Timer.getFPGATimestamp()
        self.drive_wrapper.field_centric_drive(0, 0, 2)

    def execute(self) -> None:
        set_point = self.angle - math.sin(
            (Timer.getFPGATimestamp() - self.start_time) * 16
        )
        vr = self.shimmy_pid.calculate(
            self.drive_wrapper.drivetrain.get_state().pose.rotation().radians(), set_point
        )

        self.drive_wrapper.field_centric_drive(0, 0, vr)
