import math

from phoenix6 import swerve
from wpimath._controls._controls.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from generated.tuner_constants import TunerConstants
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain
from telemetry import Telemetry

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

TURNING_PID_P = 1
TURNING_PID_I = 0
TURNING_PID_D = 0

CORRECTION_PID_P = 3
CORRECTION_PID_I = 0
CORRECTION_PID_D = 0

class DriveWrapper:
    def __init__(self, drivetrain: CommandSwerveDrivetrain):
        self.nudge_speed = NUDGE_SPEED
        self.max_speed = MAX_SPEED
        self.max_angular_speed = MAX_ANGULAR_SPEED

        self.drivetrain = drivetrain
        self.logger = Telemetry(MAX_SPEED)

        # Apply requests in field centric mode
        self.field_centric = (
            swerve.requests.FieldCentric()
            .with_deadband(DRIVE_DEADBAND)
            .with_rotational_deadband(ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        # Apply requests facing a robot centric angle
        self.turn_to_theta_drive = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(DRIVE_DEADBAND)
            .with_rotational_deadband(ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        # Other stuff
        self.brake = swerve.requests.SwerveDriveBrake()
        self.point = swerve.requests.PointWheelsAt()
        self.idle = swerve.requests.Idle()

        # Apply requests in robot centric mode
        self.robot_centric = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        # PID Stuff
        self.rotate_pid = PIDController(TURNING_PID_P, TURNING_PID_I, TURNING_PID_D)
        self.rotate_pid.enableContinuousInput(0, 2 * math.pi)

        self.drive_pid = PIDController(CORRECTION_PID_P, CORRECTION_PID_I, CORRECTION_PID_D)

        self.drivetrain.configure_auto_builder()

    def robot_centric_drive(self, vx, vy, angular):
        self.drivetrain.apply_request(lambda: self.robot_centric.with_velocity_x(vx).with_velocity_y(vy).with_rotational_rate(angular))

    def field_centric_drive(self, vx, vy):
        self.drivetrain.apply_request(lambda: self.field_centric.with_velocity_x(vx).with_velocity_y(vy))

    def theta_centric_drive(self, angle, vx, vy):
        self.drivetrain.apply_request(lambda: self.turn_to_theta_drive.with_target_direction(Rotation2d.fromDegrees(angle)).with_velocity_x(vx).with_velocity_y(vy).with_heading_pid(10, 0 ,0))
