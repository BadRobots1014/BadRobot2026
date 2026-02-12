#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from . import drivetrain_1014
import wpilib
import wpilib.drive
import wpimath
import wpimath.controller
import wpimath.filter
import commands2
from neo_bot import drivetrain_1014


class NeoBotContainer:
    def __init__(self) -> None:
        self.controller = wpilib.PS4Controller(0)
        self.drivetrain = drivetrain_1014.Drivetrain()

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Defines the default command for the drivetrain inside this method.
        """

        def drive_logic():
            x_input = -self.controller.getLeftY()
            x_input = wpimath.applyDeadband(x_input, 0.02)
            x_speed = self.xspeedLimiter.calculate(x_input) * drivetrain_1014.kMaxSpeed

            y_input = -self.controller.getLeftX()
            y_input = wpimath.applyDeadband(y_input, 0.02)
            y_speed = self.yspeedLimiter.calculate(y_input) * drivetrain_1014.kMaxSpeed

            rot_input = -self.controller.getRightX()
            rot_input = wpimath.applyDeadband(rot_input, 0.02)
            rot_speed = self.rotLimiter.calculate(rot_input) * drivetrain_1014.kMaxSpeed

            self.drivetrain.drive(x_speed, y_speed, rot_speed, True, 0.02)

        self.drivetrain.setDefaultCommand(
            commands2.RunCommand(drive_logic, self.drivetrain)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return commands2.WaitCommand(0)
