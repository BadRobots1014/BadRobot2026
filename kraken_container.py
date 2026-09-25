#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math

import commands2
from commands2 import ConditionalCommand, ParallelCommandGroup
from commands2.button import Trigger
from commands2.sysid import SysIdRoutine
from cscore import CameraServer, HttpCamera
import ntcore
from pathplannerlib.auto import (
    AutoBuilder,
    NamedCommands,
    PathConstraints,
)
from pathplannerlib.path import Translation2d
from phoenix6 import SignalLogger, swerve
import wpilib
from wpilib import DriverStation, SmartDashboard
from wpilib.interfaces import GenericHID
from wpimath.controller import PIDController
import wpimath.filter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import rotationsToRadians

from commands.extend_hopper import ExtendHopperCommand
from commands.run_conveyor import RunConveyor
from commands.run_intake import RunIntakeCommand
from commands.run_kicker import RunKickerCommand
from commands.run_shooter import RunShooterCommand
from commands.shimmy import Shimmy
from commands.strafe import Strafe
from controllers.aux_controller import AuxController
from controllers.main_controller import MainController
from controllers.test_controller import TestController
from generated.tuner_constants import TunerConstants
from hardware.impl.andymark_magnetic import AndymarkMagnetic
from hardware.impl.limelight import Limelight
from hardware.impl.pwmled import PWMLED
from hardware.impl.spark_flex_motor import SparkFlexMotorController
from hardware.impl.talonfx import TalonFXMotorController
from hardware.sim_hardware import DummyLED, DummyLimitSwitch
from robot_class import RobotClass
from routines.auto_shoot_with_intake import AutoShootWithIntake
from routines.dump_routine import DumpRoutine
from routines.goto_and_shoot import GotoAndShootRoutine
from routines.shoot_when_ready import ShootWhenReady
from subsystems import (
    conveyor,
    custom_controller,
    hopper,
    intake,
    kicker,
    pilights,
    shooter,
)
from subsystems.custom_controller import CustomController
from telemetry import Telemetry

LIMELIGHT_MAX_ANGULAR_VELOCITY = 10


class KrakenRobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.robot = RobotClass()

        # Use CommandGenericHID for controller compatibility
        self.primary = MainController(self.robot)
        self.auxiliary = AuxController(self.robot)
        self.test = TestController(self.robot)

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.ll_table = self.nt_instance.getTable("limelight")

        self.rejected_sub = self.ll_table.getBooleanTopic("rejected")
        self.rejected_pub = self.rejected_sub.publish()

        self.turn_to_theta_topic = self.nt_instance.getTable(
            "SmartDashboard"
        ).getBooleanTopic("turn_to_theta")
        self.turn_to_theta_pub = self.turn_to_theta_topic.publish()
        self.turn_to_theta_sub = self.turn_to_theta_topic.subscribe(defaultValue=False)

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # TODO: move publishing stream url to limelight
        self.camera = HttpCamera("LimelightPublisher", "http://10.10.14.12:5801")
        CameraServer.addCamera(self.camera)

        self.last_angle = Rotation2d.fromRotations(0)

    hopper_brake_mode = True

    def disabledInit(self) -> None:
        # Process fewer frames while disabled to reduce heat
        self.robot.camera_ll4.set_throttle(99)  # 99 equals 1% (process 1, skip 99)
        self.robot.hopper.set_coast()
        self.hopper_brake_mode = False

    def driveInit(self) -> None:
        self.robot.camera_ll4.set_throttle(0)  # Process all frames
        self.robot.camera_ll4.set_imu_mode(4)
        self.robot.hopper.set_brake()
        self.hopper_brake_mode = True

    def teleop_init(self) -> None:
        self.robot.camera_ll4.set_teleop_fiducial_id_filters()
        self.driveInit()

    def auto_init(self) -> None:
        self.robot.camera_ll4.set_auto_fiducial_id_filters()
        self.driveInit()

    def robotPeriodic(self) -> None:
        # All code below is limelight, so skip adding it if in sim
        if not self.robot.is_real_bot:
            return None
        SmartDashboard.putBoolean("Hopper Idle Mode", self.hopper_brake_mode)

        # Push gyro data to limelight (set to external IMU)
        robot_yaw = self.robot.drive_wrapper.drivetrain.get_state().pose.rotation().degrees()
        self.robot.camera_ll4.robot_orientation_set(robot_yaw)

        # Add vision
        cam_measurement_ll4 = self.robot.camera_ll4.get_vision_measurement()
        reject_pose_ll4 = self.robot.camera_ll4.tv_sub.get() < 1

        reject_pose_ll4 |= (
            # OR with tv rejection
            self.robot.drive_wrapper.drivetrain.pigeon2.get_angular_velocity_z_device().value
            > LIMELIGHT_MAX_ANGULAR_VELOCITY
        )

        llx = cam_measurement_ll4[0].x
        lly = cam_measurement_ll4[0].y

        posex = self.robot.drive_wrapper.drivetrain.get_state().pose.x
        posey = self.robot.drive_wrapper.drivetrain.get_state().pose.y

        x = posex - llx
        y = posey - lly

        if math.hypot(x, y) > 1:
            reject_pose_ll4 = True

        self.rejected_pub.set(reject_pose_ll4)

        if not reject_pose_ll4:
            self.robot.drive_wrapper.drivetrain.add_vision_measurement(
                cam_measurement_ll4[0], cam_measurement_ll4[1], cam_measurement_ll4[2]
            )

        return None

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        command: commands2.Command = self._auto_chooser.getSelected()
        return command
