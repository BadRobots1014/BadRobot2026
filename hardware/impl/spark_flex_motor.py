import math

import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

import phoenix6
import rev
from phoenix6.hardware import CANcoder

import phoenix6
import rev
from phoenix6.hardware import CANcoder
from hardware.base.motor import Motor
from hardware.base.encoder import Encoder
from hardware.impl.spark_relative_encoder import SparkRelativeEncoder


class SparkFlexMotor(Motor):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = rev.SparkFlex(motor_id, rev.SparkLowLevel.MotorType.kBrushless)

    def set_voltage(self, voltage: float):
        self.motor.setVoltage(voltage)

    def set_inverted(self, inverted: bool):
        self.motor.setInverted(inverted)

    def get_encoder(self) -> Encoder:
        return SparkRelativeEncoder(self.motor.getEncoder())

    # Getting active voltage
    def get_voltage(self) -> float:
        return self.motor.getBusVoltage() * self.motor.getAppliedOutput()
