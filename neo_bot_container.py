#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib
import wpilib.drive
import wpimath
import wpimath.controller
import wpimath.filter
from commands2.button import Trigger

from commands.party_mode import PartyModeCommand
from hardware.base.ledcontroller import LEDController
from hardware.impl.generic_can import GenericCAN
from hardware.impl.pwmled import PWMLED
from subsystems import drivetrain_neo
from subsystems.lights import LightSubsystem


class NeoBotContainer:
    def __init__(self) -> None:
        self.controller = wpilib.PS4Controller(0)
        self.drivetrain = drivetrain_neo.NeoDrivetrainSubsystem()

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.led_controller = PWMLED(0, 30)
        #self.lights = LightSubsystem(self.led_controller)

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Defines the default command for the drivetrain inside this method.
        """

        #Trigger(lambda: self.controller.getCircleButton()).onTrue(
        #    PartyModeCommand(self.lights)
        #)

        def drive_logic():
            x_input = -self.controller.getLeftY()
            x_input = wpimath.applyDeadband(x_input, 0.02)
            x_speed = self.xspeedLimiter.calculate(x_input) * drivetrain_neo.kMaxSpeed

            y_input = -self.controller.getLeftX()
            y_input = wpimath.applyDeadband(y_input, 0.02)
            y_speed = self.yspeedLimiter.calculate(y_input) * drivetrain_neo.kMaxSpeed

            rot_input = -self.controller.getRightX()
            rot_input = wpimath.applyDeadband(rot_input, 0.02)
            rot_speed = self.rotLimiter.calculate(rot_input) * drivetrain_neo.kMaxSpeed

            self.drivetrain.drive(x_speed, y_speed, rot_speed, True, 0.02)

        self.drivetrain.setDefaultCommand(
            commands2.RunCommand(drive_logic, self.drivetrain)
        )

    def robotPeriodic(self):
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        return commands2.WaitCommand(0)
