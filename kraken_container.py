#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib
from commands2.button import CommandGenericHID, Trigger
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import Translation2d
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.units import rotationsToRadians
from hardware.impl.limelight import Limelight
from commands.face_target import FaceTarget
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from subsystems import shooter


class KrakenRobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    # Controller axis mappings
    LEFT_X_AXIS = 0
    LEFT_Y_AXIS = 1
    RIGHT_X_AXIS = (
        2 if wpilib.RobotBase.isReal() else 4
    )  # prevent robot from spinning in real life and in sim
    RIGHT_Y_AXIS = 5
    # Controller button mappings
    CROSS_BUTTON = 1
    CIRCLE_BUTTON = 2
    L1_BUTTON = 5
    POV_UP = 0
    POV_DOWN = 180

    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        self._logger = Telemetry(self._max_speed)

        # Use CommandGenericHID for controller compatibility
        self._joystick = CommandGenericHID(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        # TODO: conditional to disable limelight in sim!!
        #
        # Initialize limelight
        self.camera = Limelight()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        SmartDashboard.putData("Pigeon", self.drivetrain.pigeon2)

        # shooter
        self._shooter = shooter.Shooter()

        # Configure the button bindings
        self.configureButtonBindings()

    # Joysticks need to be inverted or drive won't work properly

    def getLeftX(self):
        return -self._joystick.getRawAxis(self.LEFT_X_AXIS) ** 3

    def getLeftY(self):
        return -self._joystick.getRawAxis(self.LEFT_Y_AXIS) ** 3

    def getRightX(self):
        return -self._joystick.getRawAxis(self.RIGHT_X_AXIS) ** 3

    def getRightY(self):
        return -self._joystick.getRawAxis(self.RIGHT_Y_AXIS) ** 3

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        self.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        self.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        self.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )
        self._joystick.button(self.CIRCLE_BUTTON).whileTrue(
            FaceTarget(
                self.drivetrain,
                # Blue hub
                Translation2d(4.719, 3.946),
                self._drive,
                self._joystick,
                self._max_speed,
                self._max_angular_rate,
                self.LEFT_Y_AXIS,
                self.LEFT_X_AXIS,
            )
        )

        # POV up - drive forward
        self._joystick.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
            )
        )

        # POV down - drive backward
        self._joystick.povDown().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        # (self._joystick.button(8) & self._joystick.button(3)).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        # )
        # (self._joystick.button(8) & self._joystick.button(0)).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        # )
        # (self._joystick.button(9) & self._joystick.button(3)).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        # )
        # (self._joystick.button(9) & self._joystick.button(0)).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        # )

        # Reset the field-centric heading on L1 button press (left bumper)
        self._joystick.button(self.L1_BUTTON).onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
