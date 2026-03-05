#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import navx
import wpilib
import wpimath.geometry
import wpimath.kinematics

from subsystems import swervemodule_neo

MAX_SPEED = 8.0  # 3 meters per second
# Unused
# MAX_ANGULAR_SPEED = math.pi  # 1/2 rotation per second


class NeoDrivetrainSubsystem(commands2.Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        super().__init__()

        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        self.frontLeft = swervemodule_neo.SwerveModule(21, 22, 23)
        self.frontRight = swervemodule_neo.SwerveModule(11, 12, 13)
        self.backLeft = swervemodule_neo.SwerveModule(31, 32, 33)
        self.backRight = swervemodule_neo.SwerveModule(41, 42, 43)

        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        wpilib.SmartDashboard.putData("gyro", self.gyro)

        self.gyro.reset()

    def periodic(self) -> None:
        self.updateOdometry()

    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        period_seconds: float,
        field_relative: bool,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """

        wpilib.SmartDashboard.putString("xspeed", str(x_speed))
        wpilib.SmartDashboard.putString("yspeed", str(y_speed))
        wpilib.SmartDashboard.putString("rotation", str(rot))

        swerve_module_states = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        x_speed, y_speed, rot, self.gyro.getRotation2d()
                    )
                    if field_relative
                    else wpimath.kinematics.ChassisSpeeds(x_speed, y_speed, rot)
                ),
                period_seconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, MAX_SPEED
        )
        self.frontLeft.setDesiredState(swerve_module_states[0])
        self.frontRight.setDesiredState(swerve_module_states[1])
        self.backLeft.setDesiredState(swerve_module_states[2])
        self.backRight.setDesiredState(swerve_module_states[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def resetgyro(self) -> None:
        self.gyro.zeroYaw()
