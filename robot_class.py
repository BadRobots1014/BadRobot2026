import math

import wpilib
from wpilib import DriverStation
from wpimath.geometry import Translation2d
from wpimath.units import rotationsToRadians
from phoenix6 import SignalLogger, swerve

from generated.tuner_constants import TunerConstants
from hardware.impl.limelight import Limelight
from subsystems import pilights
from subsystems.conveyor import ConveyorSubsystem
from subsystems.hopper import HopperSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem
from drive_wrapper import DriveWrapper
from telemetry import Telemetry

BLUE_HUB_TRANSLATION = Translation2d(4.62, 4.04)
RED_HUB_TRANSLATION = Translation2d(11.92, 4.04)

class RobotClass:
    def __init__(self):
        self.is_real_bot = wpilib.RobotBase.isReal()

        if not self.is_real_bot:
            SignalLogger.stop()

        # Drivetrain
        self.drive_wrapper = DriveWrapper(TunerConstants.create_drivetrain())
        self.slow_mode = False

        self.shooter = ShooterSubsystem(self.is_real_bot)
        self.hopper = HopperSubsystem(self.is_real_bot)
        self.kicker = KickerSubsystem(self.is_real_bot)
        self.conveyor = ConveyorSubsystem(self.is_real_bot)
        self.intake = IntakeSubsystem(self.is_real_bot)
        self.lights = pilights.PiLights()

        self.camera_ll4 = Limelight("limelight-four", enabled=True)
        self.camera_ll2 = Limelight()

        robot_yaw = self.drive_wrapper.drivetrain.get_state().pose.rotation().degrees()
        self.camera_ll4.robot_orientation_set(robot_yaw)
        self.camera_ll4.set_imu_mode(1)
        self.camera_ll4.set_auto_fiducial_id_filters()

        self.is_blue = DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def get_hub(self) -> Translation2d:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return BLUE_HUB_TRANSLATION
        else:
            return RED_HUB_TRANSLATION

    def toggleSlowMode(self) -> None:
        self.slow_mode = not self.slow_mode