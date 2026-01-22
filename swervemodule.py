#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

import phoenix6
import rev
from phoenix6.hardware import CANcoder

#kWheelRadius = 0.0508
# kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi*2
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        :param turningEncoderChannel:   DIO input for the drive encoder channel A
        """
        self.driveMotor = rev.SparkMax(driveMotorChannel, rev.SparkMax.MotorType.kBrushless)
        self.turningMotor = rev.SparkMax(turningMotorChannel, rev.SparkMax.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = CANcoder(turningEncoderChannel)
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
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        wpilib.SmartDashboard.putData(f"pid{turningEncoderChannel}", self.turningPIDController)

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
            wpimath.geometry.Rotation2d().fromRotations(self.turningEncoder.get_position().value),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d.fromRotations(self.turningEncoder.get_position().value),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d.fromRotations(self.turningEncoder.get_position().value)

        # Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        desiredState.cosineScale(encoderRotation)

        #driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)
        #turnFeedforward = self.turnFeedforward.calculate(
        #    self.turningPIDController.getSetpoint().velocity
        #)

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), desiredState.speed
        )
        #driveOutput += driveFeedforward

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.get_position().value, desiredState.angle.radians()/(math.pi*2) #rotations
        )
        #turnOutput += turnFeedforward

        #manually clamp pid outputs to acceptable voltage
        driveOutput = max(min(driveOutput, 12), -12)
        turnOutput = max(min(turnOutput, 12), -12)

        self.driveMotor.setVoltage(driveOutput)
        self.turningMotor.setVoltage(turnOutput)