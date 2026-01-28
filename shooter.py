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

from hardware.base.encoder import Encoder
from hardware.base.motor import Motor


class Shooter:
    # def __init__(self, shoot_motor_id: int, turn_motor_id: int):
    def __init__(
        self,
        shoot_motor: Motor,
        turn_motor: Motor,
        shoot_encoder: Encoder,
        turn_encoder: Encoder,
    ):

        # self.shoot_motor = rev.SparkMax(shoot_motor_id, rev.SparkLowLevel.MotorType.kBrushless)
        # self.turn_motor = rev.SparkMax(turn_motor_id, rev.SparkLowLevel.MotorType.kBrushless)

        self.shoot_motor = shoot_motor
        self.turn_motor = turn_motor

        self.shoot_encoder = shoot_encoder
        self.turn_encoder = turn_encoder

        # self.shoot_encoder = self.shoot_motor.getEncoder()
        # self.turn_encoder = self.turn_motor.getEncoder()

        # self._shoot_sim_encoder = rev.SparkRelativeEncoderSim(self.shoot_motor)
        # self._angle_sim_encoder = rev.SparkRelativeEncoderSim(self.turn_motor)

    def set_shoot_voltage(self, volts: float):
        self.shoot_motor.set_voltage(volts)

    def set_turn_voltage(self, volts: float):
        self.turn_motor.set_voltage(volts)

    def reset_shoot(self):
        self.shoot_encoder.set_position(0)

    def reset_turn(self):
        self.turn_encoder.set_position(0)

    @property
    def shoot_distance(self) -> float:
        return self.shoot_encoder.get_position()

    @property
    def turn_distance(self) -> float:
        return self.turn_encoder.get_position()

    @property
    def shoot_voltage(self) -> float:
        return self.shoot_motor.get_voltage()

    @property
    def turn_voltage(self) -> float:
        return self.turn_motor.get_voltage()
