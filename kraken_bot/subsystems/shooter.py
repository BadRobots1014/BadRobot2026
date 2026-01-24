import math

import phoenix6
import rev
import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
from phoenix6.hardware import CANcoder


class shooter:
    def __init__(self, shootMotorChannel: int, turnMotorChannel: int):
        self._shootMotor = rev.SparkMax(
            shootMotorChannel, rev.SparkLowLevel.MotorType.kBrushless
        )
        self._turnMotor = rev.SparkMax(
            turnMotorChannel, rev.SparkLowLevel.MotorType.kBrushless
        )
        self._shootEncoder = self._shootMotor.getEncoder()
        self._turnEncoder = self._turnMotor.getEncoder()
        self._config()
        self.reset()

        self._shoot_sim_encoder = rev.SparkRelativeEncoderSim(self._shootMotor)
        self._angle_sim_encoder = rev.SparkRelativeEncoderSim(self._turnMotor)

    def set_shoot_voltage(self, volts: float):
        self._shootMotor.setVoltage(volts)

    def set_turn_voltage(self, volts: float):
        self._turnMotor.setVoltage(volts)

    def reset_shoot(self):
        self._shootEncoder.setPosition(0)

    def reset_turn(self):
        self._turnEncoder.setPosition(0)

    @property
    def shoot_distance(self) -> float:
        return self._shootEncoder.getPosition()

    @property
    def turn_distance(self) -> float:
        return self._turnEncoder.getPosition()

    @property
    def shoot_voltage(self) -> float:
        return self._shootMotor.getBusVoltage() * self._shootMotor.getAppliedOutput()

    @property
    def turn_voltage(self) -> float:
        return self._turnMotor.getBusVoltage() * self._turnMotor.getAppliedOutput()
