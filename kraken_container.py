#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from imaplib import Commands

import commands2
import rev
import wpilib
import wpimath.filter
from commands2.button import CommandGenericHID, Trigger
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import Translation2d
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.units import rotationsToRadians

from commands import run_seesaw
from commands.bang_bang_shoot import BangBangShootCommand
from commands.face_target import FaceTargetCommand
from commands.intake_demo import IntakeDemoCommand
from commands.run_intake import RunIntakeCommand
from commands.shoot import ShootCommand
from commands.shoot_kicker import ShootKickerCommand
from generated.tuner_constants import TunerConstants
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)
from hardware.impl.talonfx import TalonFXMotorController
from hardware.impl.limelight import Limelight
from hardware.impl.spark_flex_motor import SparkFlexMotorController
from hardware.impl.spark_max_motor import SparkMaxMotorController
from subsystems import music, seesaw, shooter
from subsystems.intake import IntakeSubsystem
from telemetry import Telemetry

LIMELIGHT_MAX_ANGULAR_VELOCITY = 10

# Controller axis mappings
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
RIGHT_X_AXIS = (
    2 if wpilib.RobotBase.isReal() else 4
)  # prevent robot from spinning in real life and in sim
RIGHT_Y_AXIS = 5

# Controller button mappings
CROSS_BUTTON = 2
CIRCLE_BUTTON = 3
SQUARE_BUTTON = 1
TRIANGLE_BUTTON = 4
SHARE_BUTTON = 9
L1_BUTTON = 5
R1_BUTTON = 6
L2_BUTTON = 7
R2_BUTTON = 8
POV_UP = 0
POV_RIGHT = 90
POV_LEFT = 270
POV_DOWN = 180
OPTIONS_BUTTON = 10
PADDLE_LEFT = 11
PADDLE_RIGHT = 12
HOME_BUTTON = 13
TRACKPAD = 14

# drive speeds/limits
SLOW_SPEED_JOYSTICK_MODIFIER = 0.5
MAX_SPEED = 1 * TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
NUDGE_SPEED = 0.5
MAX_ANGULAR_SPEED = rotationsToRadians(
    1.5
)  # 3/4 of a rotation per second max angular velocity
DRIVE_DEADBAND = MAX_SPEED * 0.1  # Add a 10% deadband
ANGULAR_DEADBAND = MAX_ANGULAR_SPEED * 0.1  # Add a 10% deadband

# joysticks
PRIMARY_JOYSTICK = 0
JOYSTICK_SLEW_RATE = 3

# point towards locations
BLUE_HUB_TRANSLATION = Translation2d(4.719, 3.946)

# shooter can id
MAIN_SHOOT_MOTOR_ID = 59
FOLLOWER_SHOOT_MOTOR_ID = 55
KICK_MOTOR_ID = 51
SEESAW_MOTOR_ID = 53

# intake can id
INTAKE_MOTOR_CAN_ID = 52

# pinion can id
RIGHT_PINION_ID = 45
LEFT_PINION_ID = 46


class KrakenRobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.slow_mode = False
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

        music_motors = []
        for module in self.drivetrain.modules:
            music_motors.append(module.drive_motor)
            music_motors.append(module.steer_motor)
        self.music = music.MusicSubsystem(music_motors, self.drivetrain)

        # TODO: conditional to disable limelight in sim!!
        #
        # Initialize limelight
        self.camera = Limelight()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        SmartDashboard.putData("Pigeon", self.drivetrain.pigeon2)

        self.main_shoot_motor = SparkFlexMotorController(MAIN_SHOOT_MOTOR_ID)
        self.follower_shoot_motor = SparkFlexMotorController(FOLLOWER_SHOOT_MOTOR_ID)
        self.kick_motor = SparkFlexMotorController(KICK_MOTOR_ID)
        self.seesaw_motor = SparkMaxMotorController(
            SEESAW_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushed
        )
        self.shoot_encoder = self.main_shoot_motor.get_encoder()
        self.kick_encoder = self.kick_motor.get_encoder()

        # shooter
        self._shooter = shooter.ShooterSubsystem(
            self.main_shoot_motor,
            self.follower_shoot_motor,
            self.shoot_encoder,
            self.kick_motor,
            self.kick_encoder,
        )

        self.intakeMotor = SparkFlexMotorController(INTAKE_MOTOR_CAN_ID)
        self._seesaw = seesaw.SeesawSubsystem(self.seesaw_motor)
        self.right_pinion = TalonFXMotorController(RIGHT_PINION_ID)
        self.left_pinion = TalonFXMotorController(LEFT_PINION_ID)

        self._intake = IntakeSubsystem(
            self.intakeMotor, self.right_pinion, self.left_pinion
        )

        # Configure the button bindings
        self.configureButtonBindings()

    # Joysticks need to be inverted or drive won't work properly

    def getLeftX(self):
        raw = -self._joystick.getRawAxis(LEFT_X_AXIS) ** 3
        limiter = self.left_x_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def getLeftY(self):
        raw = -self._joystick.getRawAxis(LEFT_Y_AXIS) ** 3
        limiter = self.left_y_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def getRightX(self):
        raw = -self._joystick.getRawAxis(RIGHT_X_AXIS) ** 3
        limiter = self.right_x_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def getRightY(self):
        raw = -self._joystick.getRawAxis(RIGHT_Y_AXIS) ** 3
        limiter = self.right_y_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def toggleSlowMode(self):
        self.slow_mode = not self.slow_mode

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

        # toggle slow mode
        self._joystick.button(R2_BUTTON).onTrue(
            commands2.cmd.runOnce(lambda: self.toggleSlowMode())
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        # Face target
        self._joystick.button(L2_BUTTON).whileTrue(
            FaceTargetCommand(
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

        # Run main wheel
        self._joystick.button(L1_BUTTON).whileTrue(ShootCommand(self._shooter))

        # Run kicker wheel
        self._joystick.button(R1_BUTTON).whileTrue(ShootKickerCommand(self._shooter))

        # Play music
        self._joystick.button(SHARE_BUTTON).toggleOnTrue(self.music.play_song())

        # run seesaw
        seesaw_forward = run_seesaw.RunSeesawCommand(self._seesaw, True)
        self._joystick.button(SQUARE_BUTTON).whileTrue(seesaw_forward)
        # forward
        seesaw_backward = run_seesaw.RunSeesawCommand(self._seesaw, False)
        self._joystick.button(TRIANGLE_BUTTON).whileTrue(seesaw_backward)

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

        # POV right - drive right
        self._joystick.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(
                    -NUDGE_SPEED
                )
            )
        )

        # POV up - drive forward
        self._joystick.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(
                    NUDGE_SPEED
                )
            )
        )

        IntakeWheelIn = RunIntakeCommand(self._intake, False)
        IntakeWheelOut = RunIntakeCommand(self._intake, True)
        self._joystick.button(CROSS_BUTTON).toggleOnTrue(IntakeWheelIn)
        self._joystick.button(CIRCLE_BUTTON).toggleOnTrue(IntakeWheelOut)

        # self._joystick.button(TRIANGLE_BUTTON).whileTrue(
        #    IntakeDemoCommand(self.left_pinion, self.right_pinion, True)
        # )
        # self._joystick.button(SQUARE_BUTTON).whileTrue(
        #    IntakeDemoCommand(self.left_pinion, self.right_pinion, False)
        # )

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

        # Reset the field-centric heading on Options button press
        self._joystick.button(OPTIONS_BUTTON).onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        # self.drivetrain.register_telemetry(
        #    lambda state: self._logger.telemeterize(state)
        # )

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
