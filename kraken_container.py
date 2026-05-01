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
from generated.tuner_constants import TunerConstants
from hardware.impl.andymark_magnetic import AndymarkMagnetic
from hardware.impl.limelight import Limelight
from hardware.impl.pwmled import PWMLED
from hardware.impl.spark_flex_motor import SparkFlexMotorController
from hardware.impl.talonfx import TalonFXMotorController
from hardware.sim_hardware import DummyLED, DummyLimitSwitch
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

# Controller axis mappings
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
RIGHT_X_AXIS = 4
RIGHT_Y_AXIS = 5
L2_TRIGGER_AXIS = 2
R2_TRIGGER_AXIS = 3

FLIGHT_STICK_POV_VECTORS = {
    0: (1, 0),
    45: (1, 0),
    90: (0, -1),
    135: (-1, 0),
    180: (-1, 0),
    225: (-1, 0),
    270: (0, 1),
    315: (1, 0),
}

FLIGHT_STICK_X_AXIS = 0
FLIGHT_STICK_Y_AXIS = 1
FLIGHT_STICK_YAW_AXIS = 2

AXIS_THRESHOLD_VALUE = 0.67

# Controller button mappings
CROSS_BUTTON = 1
CIRCLE_BUTTON = 2
SQUARE_BUTTON = 3
TRIANGLE_BUTTON = 4
L1_BUTTON = 5
R1_BUTTON = 6
SHARE_BUTTON = 7
OPTIONS_BUTTON = 8
L3_BUTTON = 9
R3_BUTTON = 10

POV_UP = 0
POV_RIGHT = 90
POV_LEFT = 270
POV_DOWN = 180


# drive speeds/limits
SLOW_SPEED_JOYSTICK_MODIFIER = 0.5
MAX_SPEED = 1 * TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
MAX_ACCELERATION = 3  # m/s^2
NUDGE_SPEED = 0.4 * MAX_SPEED
MAX_ANGULAR_SPEED = rotationsToRadians(
    1.5
)  # 3/4 of a rotation per second max angular velocity

MAX_ANGULAR_ACCELERATION = 10  # m/s^2
DRIVE_DEADBAND = MAX_SPEED * 0.02  # Add a 10% deadband
ANGULAR_DEADBAND = MAX_ANGULAR_SPEED * 0.02  # Add a 10% deadband
TURN_TO_THETA_DEADBAND = 0.5

# joysticks
DRIVER_PORT = 0
AUXILIARY_PORT = 1
TEST_PORT = 2
FLIGHT_STICK_PORT = 3
TURN_TO_THETA_PORT = 4

JOYSTICK_SLEW_RATE = 3

# point towards locations
BLUE_HUB_TRANSLATION = Translation2d(4.62, 4.04)
RED_HUB_TRANSLATION = Translation2d(11.92, 4.04)

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

# conveyor can id
CONVEYOR_ID = 56

# limit switch id
FORWARD_LIMIT_ID = 18

# Constraints for pathfinding
PATHFINDING_CONSTRAINTS = PathConstraints(
    MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION
)

# TODO: needs tuning

TURNING_PID_P = 0.9
TURNING_PID_I = 0
TURNING_PID_D = 0

CORRECTION_PID_P = 2
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

        if not self.is_real_bot:
            SignalLogger.stop()

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
        self._turn_to_theta_drive = (
            swerve.requests.FieldCentricFacingAngle()
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
        self._test_controller = CustomController(TEST_PORT)
        self._flight_stick = GenericHID(3)

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

        # Initialize limelight
        self.camera_ll4 = Limelight("limelight-four", enabled=True)
        self.camera_ll2 = Limelight()

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.ll_table = self.nt_instance.getTable("limelight")

        self.rejected_sub = self.ll_table.getBooleanTopic("rejected")
        self.rejected_pub = self.rejected_sub.publish()

        self.turn_to_theta_topic = self.nt_instance.getTable(
            "SmartDashboard"
        ).getBooleanTopic("turn_to_theta")
        self.turn_to_theta_pub = self.turn_to_theta_topic.publish()
        self.turn_to_theta_sub = self.turn_to_theta_topic.subscribe(defaultValue=False)

        # limit switches
        self.forward_limit_switch = (
            AndymarkMagnetic(FORWARD_LIMIT_ID)
            if self.is_real_bot
            else DummyLimitSwitch(default_state=False)
        )

        self.main_shoot_motor = SparkFlexMotorController(MAIN_SHOOT_MOTOR_ID)
        self.follower_shoot_motor = SparkFlexMotorController(FOLLOWER_SHOOT_MOTOR_ID)
        self.kick_motor = SparkFlexMotorController(KICK_MOTOR_ID)
        self.shoot_encoder = self.main_shoot_motor.get_encoder()
        self.kick_encoder = self.kick_motor.get_encoder()

        self.conveyor_motor = SparkFlexMotorController(CONVEYOR_ID)

        # shooter
        self._shooter = shooter.ShooterSubsystem(
            self.main_shoot_motor,
            self.follower_shoot_motor,
            self.shoot_encoder,
        )

        # kicker
        self._kicker = kicker.KickerSubsystem(
            self.kick_motor,
            self.kick_encoder,
        )

        self.intakeMotor = SparkFlexMotorController(INTAKE_MOTOR_CAN_ID)
        self.left_pinion = TalonFXMotorController(LEFT_PINION_ID)
        self.right_pinion = TalonFXMotorController(RIGHT_PINION_ID)

        self._intake = intake.IntakeSubsystem(
            self.intakeMotor,
        )

        self._hopper = hopper.HopperSubsystem(
            self.left_pinion.get_motor_controller(),
            self.right_pinion.get_motor_controller(),
            self.forward_limit_switch,
        )

        self._conveyor = conveyor.ConveyorSubsystem(self.conveyor_motor)

        self.drivetrain = TunerConstants.create_drivetrain()

        # takes a while and sometimes causes tests to fail maybe?
        # self.music = music.MusicSubsystem(self.drivetrain)

        # Configures limelight IMU
        robot_yaw = self.drivetrain.get_state().pose.rotation().degrees()
        self.camera_ll4.robot_orientation_set(robot_yaw)
        self.camera_ll4.set_imu_mode(1)
        self.camera_ll4.set_auto_fiducial_id_filters()

        self.rotate_pid = PIDController(TURNING_PID_P, TURNING_PID_I, TURNING_PID_D)
        self.rotate_pid.enableContinuousInput(0, 2 * math.pi)

        self.drive_pid = PIDController(
            CORRECTION_PID_P, CORRECTION_PID_I, CORRECTION_PID_D
        )

        SmartDashboard.putData("drive pid", self.drive_pid)
        SmartDashboard.putData("turn pid", self.rotate_pid)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure commands used in auto
        NamedCommands.registerCommand(
            "Extend",
            ExtendHopperCommand(self._hopper).withTimeout(1),
        )

        NamedCommands.registerCommand(
            "Slight Dump", RunIntakeCommand(self._intake, dump=True).withTimeout(0.1)
        )

        NamedCommands.registerCommand(
            "AutoIntakeShoot", AutoShootWithIntake(self._intake)
        )

        NamedCommands.registerCommand(
            "RunIntake",
            RunIntakeCommand(self._intake, dump=False).withTimeout(6),
        )

        NamedCommands.registerCommand(
            "ShootStarting8",
            ShootWhenReady(
                self._shooter, self._kicker, self._conveyor, self._intake, 3500
            ).withTimeout(3),
        )

        NamedCommands.registerCommand(
            "EmptyHopper",
            ParallelCommandGroup(
                ShootWhenReady(
                    self._shooter, self._kicker, self._conveyor, self._intake, 2700
                ),
                AutoShootWithIntake(self._intake),
            ).withTimeout(4),
        )
        NamedCommands.registerCommand(
            "GotoTowerAndShoot",
            GotoAndShootRoutine(
                self._shooter,
                self._kicker,
                self._conveyor,
                self._intake,
                self.drivetrain,
                self.drive_pid,
                self.rotate_pid,
                self.get_hub,
                self.is_blue,
            ).withTimeout(4),
        )

        NamedCommands.registerCommand(
            "ResetHeading", self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
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

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # TODO: move publishing stream url to limelight
        self.camera = HttpCamera("LimelightPublisher", "http://10.10.14.12:5801")
        CameraServer.addCamera(self.camera)

        self.last_angle = Rotation2d.fromRotations(0)

    # Joysticks need to be inverted or drive won't work properly

    def getLeftX(self) -> float:
        if DriverStation.isJoystickConnected(FLIGHT_STICK_PORT):
            raw = -(self._flight_stick.getRawAxis(FLIGHT_STICK_X_AXIS) ** 3)
        else:
            raw = -(self._primary_controller.getRawAxis(LEFT_X_AXIS) ** 3)
        if self.slow_mode:
            raw *= SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getLeftY(self) -> float:
        if DriverStation.isJoystickConnected(FLIGHT_STICK_PORT):
            raw = -(self._flight_stick.getRawAxis(FLIGHT_STICK_Y_AXIS) ** 3)
        else:
            raw = -(self._primary_controller.getRawAxis(LEFT_Y_AXIS) ** 3)
        if self.slow_mode:
            raw *= SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getRightX(self) -> float:
        if DriverStation.isJoystickConnected(FLIGHT_STICK_PORT):
            raw = -(self._flight_stick.getRawAxis(FLIGHT_STICK_YAW_AXIS) ** 3)
        else:
            raw = -(self._primary_controller.getRawAxis(RIGHT_X_AXIS) ** 3)
        if self.slow_mode:
            raw *= SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getRightY(self) -> float:
        raw = -(self._primary_controller.getRawAxis(RIGHT_Y_AXIS) ** 3)
        if self.slow_mode:
            raw *= SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getTargetAngle(self) -> Rotation2d:
        x = self.getRightX()
        y = self.getRightY()
        if math.sqrt(x * x + y * y) > TURN_TO_THETA_DEADBAND:
            self.last_angle = Rotation2d.fromRotations(math.atan2(x, y) / (2 * math.pi))
        return self.last_angle

    def getFlightStickNudgeVector(self) -> tuple[int, int]:
        return FLIGHT_STICK_POV_VECTORS[self._flight_stick.getPOV()]

    def toggleSlowMode(self) -> None:
        self.slow_mode = not self.slow_mode

    def get_hub(self) -> Translation2d:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return BLUE_HUB_TRANSLATION
        else:
            return RED_HUB_TRANSLATION

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            ConditionalCommand(
                self.drivetrain.apply_request(
                    lambda: (
                        self._turn_to_theta_drive.with_velocity_x(
                            self.getLeftY() * MAX_SPEED
                        )  # Drive forward with negative Y (forward)
                        .with_velocity_y(
                            self.getLeftX() * MAX_SPEED
                        )  # Drive left with negative X (left)
                        .with_target_direction(
                            self.getTargetAngle()
                        )  # Drive counterclockwise with negative X (left)
                        .with_heading_pid(10, 0, 0)
                    )
                ),
                self.drivetrain.apply_request(
                    lambda: (
                        self._drive.with_velocity_x(
                            self.getLeftY() * MAX_SPEED
                        )  # Drive forward with negative Y (forward)
                        .with_velocity_y(
                            self.getLeftX() * MAX_SPEED
                        )  # Drive left with negative X (left)
                        .with_rotational_rate(
                            self.getRightX() * MAX_ANGULAR_SPEED
                        )  # Drive counterclockwise with negative X (left)
                    )
                ),
                self.turn_to_theta_sub.get,
            )
        )

        Trigger(lambda: self._flight_stick.getPOV() != -1).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(
                    self.getFlightStickNudgeVector()[0] * MAX_SPEED
                ).with_velocity_y(  # Positive X to go forward
                    self.getFlightStickNudgeVector()[1] * MAX_SPEED
                )  # Positive Y to go left
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
        # self._primary_controller.create_axis(
        #     R2_TRIGGER, "Slow Mode (hold)", AXIS_THRESHOLD_VALUE
        # ).onTrue(commands2.cmd.runOnce(self.toggleSlowMode))
        # self._primary_controller.create_axis(
        #     R2_TRIGGER, "Slow Mode (hold)", AXIS_THRESHOLD_VALUE
        # ).onFalse(commands2.cmd.runOnce(self.toggleSlowMode))

        strafe_l = Strafe(
            self.drivetrain,
            self._shooter,
            self.get_hub,
            clockwise=True,
            max_angular_rate=MAX_ANGULAR_SPEED,
            rotate_pid=self.rotate_pid,
            drive_pid=self.drive_pid,
        )
        strafe_r = Strafe(
            self.drivetrain,
            self._shooter,
            self.get_hub,
            clockwise=False,
            max_angular_rate=MAX_ANGULAR_SPEED,
            rotate_pid=self.rotate_pid,
            drive_pid=self.drive_pid,
        )

        self._primary_controller.create_button(
            L1_BUTTON, "Strafe Left Around Tower"
        ).whileTrue(strafe_l)
        self._primary_controller.create_button(
            R1_BUTTON, "Strafe Right Around Tower"
        ).whileTrue(strafe_r)

        # POV up - drive forward
        self._primary_controller.create_axis(
            R2_TRIGGER_AXIS, "nudge backwards", AXIS_THRESHOLD_VALUE
        ).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(
                    NUDGE_SPEED
                ).with_velocity_y(0)
            )
        )

        # POV down - drive backward
        self._primary_controller.create_axis(
            L2_TRIGGER_AXIS, "nudge backwards", AXIS_THRESHOLD_VALUE
        ).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(
                    -NUDGE_SPEED
                ).with_velocity_y(0)
            )
        )

        # POV right - drive right
        self._primary_controller.bind_pov_right("nudge right").whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(
                    -NUDGE_SPEED
                )
            )
        )

        # POV left - drive left
        self._primary_controller.bind_pov_left("nudge left").whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(
                    NUDGE_SPEED
                )
            )
        )

        self._primary_controller.create_button(
            TRIANGLE_BUTTON, "point forward"
        ).whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._turn_to_theta_drive.with_velocity_x(
                        self.getLeftY() * MAX_SPEED
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        self.getLeftX() * MAX_SPEED
                    )  # Drive left with negative X (left)
                    .with_target_direction(
                        Rotation2d.fromDegrees(0)
                    )  # Drive counterclockwise with negative X (left)
                    .with_heading_pid(6, 0, 0)
                )
            ),
        )

        self._primary_controller.create_button(CIRCLE_BUTTON, "point right").whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._turn_to_theta_drive.with_velocity_x(
                        self.getLeftY() * MAX_SPEED
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        self.getLeftX() * MAX_SPEED
                    )  # Drive left with negative X (left)
                    .with_target_direction(
                        Rotation2d.fromDegrees(270)
                    )  # Drive counterclockwise with negative X (left)
                    .with_heading_pid(6, 0, 0)
                )
            ),
        )

        self._primary_controller.create_button(
            CROSS_BUTTON, "point backwards"
        ).whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._turn_to_theta_drive.with_velocity_x(
                        self.getLeftY() * MAX_SPEED
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        self.getLeftX() * MAX_SPEED
                    )  # Drive left with negative X (left)
                    .with_target_direction(
                        Rotation2d.fromDegrees(180)
                    )  # Drive counterclockwise with negative X (left)
                    .with_heading_pid(6, 0, 0)
                )
            ),
        )

        self._primary_controller.create_button(SQUARE_BUTTON, "point left").whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._turn_to_theta_drive.with_velocity_x(
                        self.getLeftY() * MAX_SPEED
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        self.getLeftX() * MAX_SPEED
                    )  # Drive left with negative X (left)
                    .with_target_direction(
                        Rotation2d.fromDegrees(90)
                    )  # Drive counterclockwise with negative X (left)
                    .with_heading_pid(6, 0, 0)
                )
            ),
        )

        # Reset the field-centric heading on Options button press
        self._primary_controller.create_button(OPTIONS_BUTTON, "Reset Heading").onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric).andThen(
                commands2.InstantCommand(self.camera_ll4.set_imu_mode(1))
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(
                doesRunWhenDisabled=True
            )
        )

        self._primary_controller.bind_pov_down("waggle").whileTrue(
            Shimmy(self.drivetrain)
        )

        # AUX CONTROLLER -------------------------------------------------------------------------------

        # manual extend
        self._auxiliary_controller.bind_pov_up("Manual extend hopper").whileTrue(
            ExtendHopperCommand(self._hopper)
        )

        # Spin up shooter L2
        self._auxiliary_controller.create_axis(
            L2_TRIGGER_AXIS, "shoot when ready", AXIS_THRESHOLD_VALUE
        ).whileTrue(
            ShootWhenReady(
                self._shooter, self._kicker, self._conveyor, self._intake, rpm=3300
            ),
        )

        # Run kicker wheel when ready R2
        self._auxiliary_controller.create_axis(
            R2_TRIGGER_AXIS,
            "goto and shoot when ready (dangerous)",
            AXIS_THRESHOLD_VALUE,
        ).whileTrue(
            GotoAndShootRoutine(
                self._shooter,
                self._kicker,
                self._conveyor,
                self._intake,
                self.drivetrain,
                self.drive_pid,
                self.rotate_pid,
                self.get_hub,
                self.is_blue,
            )
            # goto_radius
        )

        # uncomment if you want to use the regular kicker command
        # self._auxiliary_controller.create_button(
        #     R1_BUTTON,
        #     "Run kicker wheel",
        #     ).whileTrue(ShootKickerCommand(self._kicker, invert=False))

        self._auxiliary_controller.create_button(
            L1_BUTTON, "shoot when ready (rpm=None)"
        ).whileTrue(
            ShootWhenReady(
                self._shooter, self._kicker, self._conveyor, self._intake, rpm=None
            )
        )

        # Intake wheel in (HOLD)
        intake_wheel_in = RunIntakeCommand(self._intake, dump=False)
        self._auxiliary_controller.create_button(
            CROSS_BUTTON, "Intake wheel in"
        ).whileTrue(ExtendHopperCommand(self._hopper).andThen(intake_wheel_in))
        # Intake wheel dump (HOLD)
        intake_wheel_out = DumpRoutine(self._intake, self._kicker, self._conveyor)
        self._auxiliary_controller.create_button(
            CIRCLE_BUTTON, "Intake wheel dump"
        ).whileTrue(intake_wheel_out)

        # Intake wheel down up
        self._auxiliary_controller.create_button(
            SQUARE_BUTTON, "intake pulse"
        ).whileTrue(AutoShootWithIntake(self._intake))

        # Party Mode
        # self._auxiliary_controller.button(SHARE_BUTTON).toggleOnTrue(
        #    PartyModeCommand(self._lights, self.music)
        # )

        # test controls -------------------------------------------------------

        self._test_controller.create_axis(
            R2_TRIGGER_AXIS, "shoot", AXIS_THRESHOLD_VALUE
        ).whileTrue(RunShooterCommand(self._shooter, rpm=3500))
        self._test_controller.create_axis(
            L2_TRIGGER_AXIS, "extend hopper test", AXIS_THRESHOLD_VALUE
        ).whileTrue(ExtendHopperCommand(self._hopper))
        self._test_controller.create_button(L1_BUTTON, "kicker").whileTrue(
            RunKickerCommand(self._kicker, invert=False)
        )
        self._test_controller.create_button(R1_BUTTON, "kicker invert").whileTrue(
            RunKickerCommand(self._kicker, invert=True)
        )
        self._test_controller.create_button(TRIANGLE_BUTTON, "conveyor").whileTrue(
            RunConveyor(self._conveyor, shoot_direction=True)
        )
        self._test_controller.create_button(SQUARE_BUTTON, "conveyor invert").whileTrue(
            RunConveyor(self._conveyor, shoot_direction=False)
        )
        self._test_controller.create_button(CROSS_BUTTON, "intake").whileTrue(
            RunIntakeCommand(self._intake, dump=False)
        )
        self._test_controller.create_button(CIRCLE_BUTTON, "intake invert").whileTrue(
            RunIntakeCommand(self._intake, dump=True)
        )

        self.drivetrain.register_telemetry(self._logger.telemeterize)
        custom_controller.write_binds()

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._test_controller.button(SHARE_BUTTON)).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._test_controller.button(OPTIONS_BUTTON)).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._test_controller.button(L3_BUTTON)).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._test_controller.button(R3_BUTTON)).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

    hopper_brake_mode = True

    def disabledInit(self) -> None:
        # Process fewer frames while disabled to reduce heat
        self.camera_ll4.set_throttle(99)  # 99 equals 1% (process 1, skip 99)
        self._hopper.set_coast()
        self.hopper_brake_mode = False
        self.camera_ll4.check_fms_capture_replay()

    def driveInit(self) -> None:
        self.camera_ll4.check_fms_enable_replay()
        self.camera_ll4.set_throttle(0)  # Process all frames
        self.camera_ll4.set_imu_mode(4)
        self._hopper.set_brake()
        self.hopper_brake_mode = True

    def teleop_init(self) -> None:
        self.camera_ll4.set_teleop_fiducial_id_filters()
        self.driveInit()
        self.camera_ll4.check_fms_capture_replay()

    def auto_init(self) -> None:
        self.camera_ll4.set_auto_fiducial_id_filters()
        self.driveInit()

    def robotPeriodic(self) -> None:
        # All code below is limelight, so skip adding it if in sim
        if not self.is_real_bot:
            return None
        SmartDashboard.putBoolean("Hopper Idle Mode", self.hopper_brake_mode)

        # Push gyro data to limelight (set to external IMU)
        robot_yaw = self.drivetrain.get_state().pose.rotation().degrees()
        self.camera_ll4.robot_orientation_set(robot_yaw)

        # Add vision
        cam_measurement_ll4 = self.camera_ll4.get_vision_measurement()
        reject_pose_ll4 = self.camera_ll4.tv_sub.get() < 1

        reject_pose_ll4 |= (
            # OR with tv rejection
            self.drivetrain.pigeon2.get_angular_velocity_z_device().value
            > LIMELIGHT_MAX_ANGULAR_VELOCITY
        )

        # reject before hopper is out
        # reject_pose_ll4 |= not self._hopper.has_hopper_extended

        # llx = cam_measurement_ll4[0].x
        # lly = cam_measurement_ll4[0].y
        #
        # posex = self.drivetrain.get_state().pose.x
        # posey = self.drivetrain.get_state().pose.y
        #
        # x = posex - llx
        # y = posey - lly

        # if math.hypot(x, y) > 1:
        #     reject_pose_ll4 = True

        self.rejected_pub.set(reject_pose_ll4)

        modified_stddevs = (
            cam_measurement_ll4[2][0] / 3,
            cam_measurement_ll4[2][1] / 3,
            cam_measurement_ll4[2][2],
        )

        if not reject_pose_ll4:
            self.drivetrain.add_vision_measurement(
                cam_measurement_ll4[0], cam_measurement_ll4[1], modified_stddevs
            )

        return None

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        command: commands2.Command = self._auto_chooser.getSelected()
        return command
