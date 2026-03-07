#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2.button import Trigger
import wpilib
import wpilib.drive
import wpimath
import wpimath.controller
import wpimath.filter
from commands2.button import Trigger

from wpilib import Color

from hardware.impl.pwmled import PWMLED
from subsystems import drivetrain_neo, lights


class NeoBotContainer:
    def __init__(self) -> None:
        self.controller = wpilib.PS4Controller(0)
        self.drivetrain = drivetrain_neo.NeoDrivetrainSubsystem()

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.led_controller = PWMLED(8, 108)
        self.lights = lights.LightSubsystem(self.led_controller)

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Defines the default command for the drivetrain inside this method.
        """

        Trigger(lambda: self.controller.getShareButton()).onTrue(
            commands2.cmd.runOnce(
                lambda: self.lights.set_rainbow(255, 150, 2), self.lights
            )
        ).onFalse(commands2.cmd.runOnce(self.lights.set_default, self.lights))

        Trigger(self.controller.getOptionsButton).onTrue(
            commands2.cmd.runOnce(self.drivetrain.resetgyro)
        )

        def drive_logic() -> None:
            x_input = -self.controller.getLeftY()
            x_input = wpimath.applyDeadband(x_input, 0.02)
            x_speed = self.xspeedLimiter.calculate(x_input) * drivetrain_neo.MAX_SPEED

            y_input = -self.controller.getLeftX()
            y_input = wpimath.applyDeadband(y_input, 0.02)
            y_speed = self.yspeedLimiter.calculate(y_input) * drivetrain_neo.MAX_SPEED

            rot_input = -self.controller.getRightX()
            rot_input = wpimath.applyDeadband(rot_input, 0.02)
            rot_speed = self.rotLimiter.calculate(rot_input) * drivetrain_neo.MAX_SPEED

            self.drivetrain.drive(
                x_speed, y_speed, rot_speed, period_seconds=0.02, field_relative=True
            )

        self.drivetrain.setDefaultCommand(
            commands2.RunCommand(drive_logic, self.drivetrain)
        )

    def robotPeriodic(self) -> None:
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        return commands2.WaitCommand(0)
