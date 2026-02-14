import rev

from hardware.base.encoder import Encoder
from hardware.base.motor import Motor
from hardware.impl.spark_relative_encoder import SparkRelativeEncoder


class SparkFlexMotor(Motor):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = rev.SparkFlex(motor_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.controller = self.motor.getClosedLoopController()

    def set_voltage(self, voltage: float):
        self.motor.setVoltage(voltage)

    def set_inverted(self, inverted: bool):
        self.motor.setInverted(inverted)

    def set_velocity(self, velocity: float):
        self.controller.setSetpoint(
            velocity, rev._rev.SparkLowLevel.ControlType.kVelocity
        )

    def get_encoder(self) -> Encoder:
        return SparkRelativeEncoder(self.motor.getEncoder())

    # Getting active voltage
    def get_voltage(self) -> float:
        return self.motor.getBusVoltage() * self.motor.getAppliedOutput()

    def get_forward_limit(self) -> bool:
        return self.motor.getForwardLimitSwitch().get()

    def get_backward_limit(self) -> bool:
        return self.motor.getReverseLimitSwitch().get()
