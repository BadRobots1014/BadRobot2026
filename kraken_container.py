#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib
import wpimath.filter
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

LIMELIGHT_MAX_ANGULAR_VELOCITY = 10

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

# drive speeds/limits
MAX_SPEED = (
    1.0 * TunerConstants.speed_at_12_volts
)  # speed_at_12_volts desired top speed
NUDGE_SPEED = 0.5
MAX_ANGULAR_SPEED = rotationsToRadians(
    0.75
)  # 3/4 of a rotation per second max angular velocity
DRIVE_DEADBAND = MAX_SPEED * 0.1  # Add a 10% deadband
ANGULAR_DEADBAND = MAX_ANGULAR_SPEED * 0.1  # Add a 10% deadband

# joysticks
PRIMARY_JOYSTICK = 0
JOYSTICK_SLEW_RATE = 3

# point towards locations
BLUE_HUB_TRANSLATION = Translation2d(4.719, 3.946)


class KrakenRobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(DRIVE_DEADBAND)
            .with_rotational_deadband(ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        self._logger = Telemetry(MAX_SPEED)

        # Use CommandGenericHID for controller compatibility
        self._joystick = CommandGenericHID(PRIMARY_JOYSTICK)

        self.left_x_speed_limiter = wpimath.filter.SlewRateLimiter(
            JOYSTICK_SLEW_RATE, -JOYSTICK_SLEW_RATE
        )
        self.left_y_speed_limiter = wpimath.filter.SlewRateLimiter(
            JOYSTICK_SLEW_RATE, -JOYSTICK_SLEW_RATE
        )
        self.right_x_speed_limiter = wpimath.filter.SlewRateLimiter(
            JOYSTICK_SLEW_RATE, -JOYSTICK_SLEW_RATE
        )
        self.right_y_speed_limiter = wpimath.filter.SlewRateLimiter(
            JOYSTICK_SLEW_RATE, -JOYSTICK_SLEW_RATE
        )

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
        raw = -self._joystick.getRawAxis(LEFT_X_AXIS)
        return self.left_x_speed_limiter.calculate(raw)

    def getLeftY(self):
        raw = -self._joystick.getRawAxis(LEFT_Y_AXIS)
        return self.left_y_speed_limiter.calculate(raw)

    def getRightX(self):
        raw = -self._joystick.getRawAxis(RIGHT_X_AXIS)
        return self.right_x_speed_limiter.calculate(raw)

    def getRightY(self):
        raw = -self._joystick.getRawAxis(RIGHT_Y_AXIS)
        return self.right_y_speed_limiter.calculate(raw)

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
                        self.getLeftY() * MAX_SPEED
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        self.getLeftX() * MAX_SPEED
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        self.getRightX() * MAX_SPEED
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
        self._joystick.button(CIRCLE_BUTTON).whileTrue(
            FaceTarget(
                self.drivetrain,
                BLUE_HUB_TRANSLATION,
                self._drive,
                self._joystick,
                MAX_SPEED,
                MAX_ANGULAR_SPEED,
                LEFT_Y_AXIS,
                LEFT_X_AXIS,
            )
        )

        # POV up - drive forward
        self._joystick.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(
                    NUDGE_SPEED
                ).with_velocity_y(0)
            )
        )

        # POV down - drive backward
        self._joystick.povDown().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(
                    -NUDGE_SPEED
                ).with_velocity_y(0)
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
        self._joystick.button(L1_BUTTON).onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def robotPeriodic(self):
        # Push gyro data to limelight (set to external IMU)
        robot_yaw = self.drivetrain.get_state().pose.rotation().degrees()
        self.camera.robot_orientation_set(robot_yaw)

        # Add vision
        cam_measurement = self.camera.get_vision_measurement()
        reject_pose = self.camera.tv_sub.get() < 1
        if not reject_pose:
            # TODO: change the angular velocity after limelight upgrade
            reject_pose = (
                self.drivetrain.pigeon2.get_angular_velocity_z_device().value
                > LIMELIGHT_MAX_ANGULAR_VELOCITY
            )
        if not reject_pose:
            self.drivetrain.add_vision_measurement(
                cam_measurement[0], cam_measurement[1], cam_measurement[2]
            )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
