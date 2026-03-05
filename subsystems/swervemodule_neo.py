#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

from phoenix6.hardware import CANcoder
import rev
import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

# kWheelRadius = 0.0508
# kEncoderResolution = 4096
MODULE_MAX_ANGULAR_VELOCITY = math.pi * 2
MODULE_MAX_ANGULAR_ACCELERATION = math.tau

# Values from 2025 Code
WHEEL_DIAMETER_METERS = 0.095
DRIVE_MOTOR_GEAR_RATIO = 1 / 8.14
DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * math.pi * WHEEL_DIAMETER_METERS
DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60


class SwerveModule:
    def __init__(
        self,
        drive_motor_channel: int,
        turning_motor_channel: int,
        turning_encoder_channel: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        :param turningEncoderChannel:   DIO input for the drive encoder channel A
        """
        self.driveConfig = rev.SparkMaxConfig()

        self.driveConfig.encoder.positionConversionFactor(DRIVE_ENCODER_ROT_2_METER)
        self.driveConfig.encoder.velocityConversionFactor(
            DRIVE_ENCODER_RPM_2_METER_PER_SEC
        )

        self.driveMotor = rev.SparkMax(
            drive_motor_channel, rev.SparkMax.MotorType.kBrushless
        )
        self.turningMotor = rev.SparkMax(
            turning_motor_channel, rev.SparkMax.MotorType.kBrushless
        )

        self.driveMotor.configure(
            self.driveConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = CANcoder(turning_encoder_channel)
        self.turningMotor.setInverted(True)
        self.driveMotor.setInverted(True)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(1, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            50,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                MODULE_MAX_ANGULAR_VELOCITY,
                MODULE_MAX_ANGULAR_ACCELERATION,
            ),
        )

        wpilib.SmartDashboard.putData(
            f"pid{turning_encoder_channel}", self.turningPIDController
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.
        # self.driveEncoder.(
        #     math.tau * kWheelRadius / kEncoderResolution
        # )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.
        # self.turningEncoder.setDistancePerPulse(math.tau / kEncoderResolution)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(0, 1)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d().fromRotations(
                self.turningEncoder.get_position().value
            ),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d.fromRotations(
                self.turningEncoder.get_position().value
            ),
        )

    def setDesiredState(
        self, desired_state: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoder_rotation = wpimath.geometry.Rotation2d.fromRotations(
            self.turningEncoder.get_position().value
        )

        # Optimize the reference state to avoid spinning further than 90 degrees
        desired_state.optimize(encoder_rotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        desired_state.cosineScale(encoder_rotation)

        # driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)
        # turnFeedforward = self.turnFeedforward.calculate(
        #    self.turningPIDController.getSetpoint().velocity
        # )

        # Calculate the drive output from the drive PID controller.
        drive_output = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), desired_state.speed
        )
        # driveOutput += driveFeedforward

        # Calculate the turning motor output from the turning PID controller.
        turn_output = self.turningPIDController.calculate(
            self.turningEncoder.get_position().value,
            desired_state.angle.radians() / (math.pi * 2),  # rotations
        )
        # turnOutput += turnFeedforward

        # manually clamp pid outputs to acceptable voltage
        drive_output = max(min(drive_output, 12), -12)
        turn_output = max(min(turn_output, 12), -12)

        self.driveMotor.setVoltage(drive_output)
        self.turningMotor.setVoltage(turn_output)
