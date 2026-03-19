#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math

import commands2
from commands2.button import Trigger
from cscore import CameraServer, HttpCamera
from pathplannerlib.auto import (
    AutoBuilder,
    NamedCommands,
    PathConstraints,
    PathPlannerPath,
)
from pathplannerlib.path import Translation2d
from phoenix6 import swerve
import wpilib
from wpilib import DriverStation, SmartDashboard
from wpimath.controller import PIDController
import wpimath.filter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import rotationsToRadians

from commands.extend_hopper import ExtendHopperCommand
from commands.face_target import FaceTargetCommand
from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from commands.party_mode import PartyModeCommand
from commands.shoot_kicker import ShootKickerCommand
from commands.strafe import Strafe
from generated.tuner_constants import TunerConstants
from hardware.impl.andymark_magnetic import AndymarkMagnetic
from hardware.impl.limelight import Limelight
from hardware.impl.pwmled import PWMLED
from hardware.impl.spark_flex_motor import SparkFlexMotorController
from hardware.impl.talonfx import TalonFXMotorController
from hardware.sim_hardware import DummyLED, DummyLimitSwitch, patch_limelight
from routines.dump_routine import DumpRoutine
from routines.extend_and_intake import ExtendAndIntakeRoutine
from routines.goto_and_shoot import GotoAndShoot
from subsystems import music, pilights, shooter, talonFXIntake
from subsystems.custom_controller import CustomController
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
MAX_ACCELERATION = 3  # m/s^2
NUDGE_SPEED = 0.7
MAX_ANGULAR_SPEED = rotationsToRadians(
    1.5
)  # 3/4 of a rotation per second max angular velocity

MAX_ANGULAR_ACCELERATION = 10  # m/s^2
DRIVE_DEADBAND = MAX_SPEED * 0.1  # Add a 10% deadband
ANGULAR_DEADBAND = MAX_ANGULAR_SPEED * 0.1  # Add a 10% deadband

# joysticks
DRIVER_PORT = 0
AUXILIARY_PORT = 1
JOYSTICK_SLEW_RATE = 3

# point towards locations
BLUE_HUB_TRANSLATION = Translation2d(4.62, 4.04)
RED_HUB_TRANSLATION = Translation2d(-4.62, -4.04)

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

# limit switch id
FORWARD_LIMIT_ID = 18
BACKWARD_LIMIT_ID = 19

# Constraints for pathfinding
PATHFINDING_CONSTRAINTS = PathConstraints(
    MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION
)

# TODO: needs tuning

TURNING_PID_P = 1
TURNING_PID_I = 0
TURNING_PID_D = 0

CORRECTION_PID_P = 3
CORRECTION_PID_I = 0
CORRECTION_PID_D = 0


class KrakenRobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Used for patching components in sim
        self.is_real_bot = wpilib.RobotBase.isReal()
        self.is_blue = DriverStation.getAlliance() == DriverStation.Alliance.kBlue

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
        self._primary_controller = CustomController(DRIVER_PORT)
        self._auxiliary_controller = CustomController(AUXILIARY_PORT)

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

        self.led_controller = PWMLED(0, 60) if self.is_real_bot else DummyLED(0, 60)
        self._lights = pilights.PiLights()

        # TODO: conditional to disable limelight in sim!!
        #
        # Initialize limelight
        self.camera_ll4 = Limelight("limelight-four", enabled=True)
        self.camera_ll2 = Limelight()

        if not self.is_real_bot:
            patch_limelight("limelight-four")
            patch_limelight("limelight")

        # limit switches
        self.forward_limit_switch = (
            AndymarkMagnetic(FORWARD_LIMIT_ID)
            if self.is_real_bot
            else DummyLimitSwitch(default_state=False)
        )
        self.backward_limit_switch = (
            AndymarkMagnetic(BACKWARD_LIMIT_ID)
            if self.is_real_bot
            else DummyLimitSwitch(default_state=True)
        )

        self.main_shoot_motor = SparkFlexMotorController(MAIN_SHOOT_MOTOR_ID)
        self.follower_shoot_motor = SparkFlexMotorController(FOLLOWER_SHOOT_MOTOR_ID)
        self.kick_motor = SparkFlexMotorController(KICK_MOTOR_ID)
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
        self.left_pinion = TalonFXMotorController(LEFT_PINION_ID)
        self.right_pinion = TalonFXMotorController(RIGHT_PINION_ID)

        self._talonIntake = talonFXIntake.TalonIntakeSubsystem(
            self.intakeMotor,
            self.left_pinion.get_motor_controller(),
            self.right_pinion.get_motor_controller(),
            self.forward_limit_switch,
            self.backward_limit_switch,
        )

        self.drivetrain = TunerConstants.create_drivetrain()

        self.music = music.MusicSubsystem(self.drivetrain)

        # Configures limelight IMU
        robot_yaw = self.drivetrain.get_state().pose.rotation().degrees()
        self.camera_ll4.robot_orientation_set(robot_yaw)
        self.camera_ll4.set_imu_mode(1)

        self.rotate_pid = PIDController(TURNING_PID_P, TURNING_PID_I, TURNING_PID_D)
        self.rotate_pid.enableContinuousInput(0, 2 * math.pi)

        self.drive_pid = PIDController(
            CORRECTION_PID_P, CORRECTION_PID_I, CORRECTION_PID_D
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure commands used in auto
        NamedCommands.registerCommand(
            "ExtendAndIntake",
            ExtendAndIntakeRoutine(self._talonIntake, self._lights),
        )
        NamedCommands.registerCommand(
            "RetractHopper",
            ExtendHopperCommand(self._talonIntake, self._lights, extend=False),
        )
        NamedCommands.registerCommand(
            "GotoTowerAndShoot",
            GotoAndShoot(
                self._shooter,
                self.drivetrain,
                self._lights,
                self.drive_pid,
                self.rotate_pid,
                BLUE_HUB_TRANSLATION,
            ),
        )

        # Run auto builder
        self.drivetrain._configure_auto_builder()

        # Configure commands used in auto that require AutoBuilder
        NamedCommands.registerCommand(
            "GotoHumanFeed",
            AutoBuilder.pathfindToPose(
                Pose2d(0.6, 0.65, Rotation2d.fromDegrees(0)), PATHFINDING_CONSTRAINTS
            ),
        )
        NamedCommands.registerCommand(
            "GotoLeftAndPickup",
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Pickup Left"), PATHFINDING_CONSTRAINTS
            ),
        )
        NamedCommands.registerCommand(
            "GotoRightAndPickup",
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Pickup Right"), PATHFINDING_CONSTRAINTS
            ),
        )

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        SmartDashboard.putData("Pigeon", self.drivetrain.pigeon2)
        SmartDashboard.putData(
            "Command Scheduler", commands2.CommandScheduler.getInstance()
        )

        # TODO: move publishing stream url to limelight
        self.camera = HttpCamera("Limelight-stream", "http://limelight.local:5800")
        CameraServer.addCamera(self.camera)

    # Joysticks need to be inverted or drive won't work properly

    def getLeftX(self) -> float:
        raw = -(self._primary_controller.getRawAxis(LEFT_X_AXIS) ** 3)
        limiter = self.left_x_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def getLeftY(self) -> float:
        raw = -(self._primary_controller.getRawAxis(LEFT_Y_AXIS) ** 3)
        limiter = self.left_y_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def getRightX(self) -> float:
        raw = -(self._primary_controller.getRawAxis(RIGHT_X_AXIS) ** 3)
        limiter = self.right_x_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def getRightY(self) -> float:
        raw = -(self._primary_controller.getRawAxis(RIGHT_Y_AXIS) ** 3)
        limiter = self.right_y_speed_limiter.calculate(raw)
        if self.slow_mode:
            limiter *= SLOW_SPEED_JOYSTICK_MODIFIER
        return limiter

    def toggleSlowMode(self) -> None:
        self.slow_mode = not self.slow_mode
        if self.slow_mode:
            self._lights.set_state(pilights.LEDState.SLOW_MODE)

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

        # PRIMARY CONTROLLER ---------------------------------------------------------------------------

        # Slow mode (hold)
        # Test pathfinding
        # self._primary_controller.button(1).onTrue(
        #     AutoBuilder.pathfindToPose(
        #         Pose2d(1, 1, Rotation2d.fromDegrees(180)),
        #         PATHFINDING_CONSTRAINTS,
        #         goal_end_vel=0,
        #     )
        # )

        # toggle slow mode
        self._primary_controller.create_button(R2_BUTTON, "Toggle Slow Mode").onTrue(
            commands2.cmd.runOnce(self.toggleSlowMode)
        )
        self._primary_controller.create_button(R2_BUTTON, "Toggle Slow Mode").onFalse(
            commands2.cmd.runOnce(self.toggleSlowMode)
        )

        strafe_l = Strafe(
            self.drivetrain,
            self._shooter,
            self._lights,
            BLUE_HUB_TRANSLATION if self.is_blue else RED_HUB_TRANSLATION,
            clockwise=True,
            max_angular_rate=MAX_ANGULAR_SPEED,
            rotate_pid=self.rotate_pid,
            drive_pid=self.drive_pid,
        )
        strafe_r = Strafe(
            self.drivetrain,
            self._shooter,
            self._lights,
            BLUE_HUB_TRANSLATION if self.is_blue else RED_HUB_TRANSLATION,
            clockwise=False,
            max_angular_rate=MAX_ANGULAR_SPEED,
            rotate_pid=self.rotate_pid,
            drive_pid=self.drive_pid,
        )

        self._primary_controller.button(L1_BUTTON).whileTrue(strafe_l)
        self._primary_controller.button(R1_BUTTON).whileTrue(strafe_r)

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(
                doesRunWhenDisabled=True
            )
        )

        # Face target
        self._primary_controller.create_button(L2_BUTTON, "Face Target").whileTrue(
            FaceTargetCommand(
                self.drivetrain,
                BLUE_HUB_TRANSLATION if self.is_blue else RED_HUB_TRANSLATION,
                self._drive,
                self._primary_controller,
                MAX_SPEED,
                MAX_ANGULAR_SPEED,
                LEFT_Y_AXIS,
                LEFT_X_AXIS,
            )
        )

        # POV up - drive forward
        self._primary_controller.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(
                    NUDGE_SPEED
                ).with_velocity_y(0)
            )
        )

        # POV down - drive backward
        self._primary_controller.povDown().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(
                    -NUDGE_SPEED
                ).with_velocity_y(0)
            )
        )

        # POV right - drive right
        self._primary_controller.povRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(
                    -NUDGE_SPEED
                )
            )
        )

        # POV left - drive left
        self._primary_controller.povLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(
                    NUDGE_SPEED
                )
            )
        )

        # Extend hopper Triangle (HOLD)
        self._primary_controller.button(TRIANGLE_BUTTON).whileTrue(
            ExtendHopperCommand(self._talonIntake, self._lights, extend=True)
        )

        # Retract hopper Square (HOLD)
        self._primary_controller.button(SQUARE_BUTTON).whileTrue(
            ExtendHopperCommand(self._talonIntake, self._lights, extend=False)
        )

        # Reset the field-centric heading on Options button press
        self._primary_controller.button(OPTIONS_BUTTON).onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric).andThen(
                commands2.InstantCommand(self.camera_ll4.set_imu_mode(1))
            )
        )

        # AUX CONTROLLER -------------------------------------------------------------------------------

        # Spin up shooter L2
        self._auxiliary_controller.create_button(L2_BUTTON, "Run main wheel").whileTrue(
            KickerShootWhenReadyCommand(self._shooter, self._lights, rpm=3300),
        )

        # Run kicker wheel when ready R2
        self._auxiliary_controller.create_button(
            R2_BUTTON,
            "Run kicker wheel when ready",
        ).whileTrue(
            GotoAndShoot(
                self._shooter,
                self.drivetrain,
                self._lights,
                self.drive_pid,
                self.rotate_pid,
                BLUE_HUB_TRANSLATION if self.is_blue else RED_HUB_TRANSLATION,
            )
        )

        # uncomment if you want to use the regular kicker command
        # self._auxiliary_controller.create_button(
        #     R1_BUTTON,
        #     "Run kicker wheel",
        #     ).whileTrue(ShootKickerCommand(self._shooter, invert=False))

        # Run kicker wheel backwards R1
        self._auxiliary_controller.create_button(
            R1_BUTTON, "Run kicker wheel inverted"
        ).whileTrue(ShootKickerCommand(self._shooter, invert=True))

        # # Jiggle L1
        # self._auxiliary_controller.create_button(L1_BUTTON, "Jiggle").whileTrue(
        #     JiggleCommand(self._talonIntake, self._lights)
        # )

        self._auxiliary_controller.create_button(L1_BUTTON, "kick maual").whileTrue(
            ShootKickerCommand(self._shooter, invert=False)
        )

        # Extend hopper Triangle (HOLD)
        self._auxiliary_controller.button(TRIANGLE_BUTTON).whileTrue(
            ExtendHopperCommand(self._talonIntake, self._lights, extend=True)
        )

        # Retract hopper Square (HOLD)
        self._auxiliary_controller.button(SQUARE_BUTTON).whileTrue(
            ExtendHopperCommand(self._talonIntake, self._lights, extend=False)
        )

        # Intake wheel in (TOGGLE)
        intake_wheel_in = ExtendAndIntakeRoutine(self._talonIntake, self._lights)
        self._auxiliary_controller.button(CROSS_BUTTON).toggleOnTrue(intake_wheel_in)

        # Intake wheel dump (TOGGLE)
        intake_wheel_out = DumpRoutine(self._talonIntake, self._shooter, self._lights)
        self._auxiliary_controller.button(CIRCLE_BUTTON).toggleOnTrue(intake_wheel_out)

        # Party Mode
        self._auxiliary_controller.button(SHARE_BUTTON).toggleOnTrue(
            PartyModeCommand(self._lights, self.music)
        )

        self.drivetrain.register_telemetry(self._logger.telemeterize)

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

    def driveInit(self) -> None:
        self.camera_ll4.set_imu_mode(4)

    def robotPeriodic(self) -> None:
        # All code below is limelight, so skip adding it if in sim
        if not self.is_real_bot:
            return None

        # Push gyro data to limelight (set to external IMU)
        robot_yaw = self.drivetrain.get_state().pose.rotation().degrees()
        self.camera_ll4.robot_orientation_set(robot_yaw)
        # self.camera_ll2.robot_orientation_set(robot_yaw)

        # Add vision
        cam_measurement_ll4 = self.camera_ll4.get_vision_measurement()
        reject_pose_ll4 = self.camera_ll4.tv_sub.get() < 1

        # cam_measurement_ll2 = self.camera_ll2.get_vision_measurement()
        # reject_pose_ll2 = self.camera_ll2.tv_sub.get() < 1

        reject_pose_ll4 |= (
            # OR with tv rejection
            self.drivetrain.pigeon2.get_angular_velocity_z_device().value
            > LIMELIGHT_MAX_ANGULAR_VELOCITY
        )
        # reject_pose_ll2 = False

        if not reject_pose_ll4:
            self.drivetrain.add_vision_measurement(
                cam_measurement_ll4[0], cam_measurement_ll4[1], cam_measurement_ll4[2]
            )

        # if not reject_pose_ll2:
        #    self.drivetrain.add_vision_measurement(
        #        cam_measurement_ll2[0], cam_measurement_ll2[1], cam_measurement_ll2[2]
        #    )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
