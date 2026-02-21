import rev

from hardware.base.encoder import Encoder
from hardware.base.motor import Motor
from hardware.impl.spark_relative_encoder import SparkRelativeEncoder
import phoenix6


class Kraken(Motor):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = phoenix6.hardware.talon_fx.TalonFX(motor_id)

    def set_voltage(self, voltage: float):
        self.motor.setVoltage(voltage)

    def set_inverted(self, inverted: bool):
        configuration = (
            phoenix6.hardware.talon_fx.configs.TalonFXConfiguration().with_motor_output(
                phoenix6.hardware.talon_fx.configs.MotorOutputConfigs().with_inverted(
                    phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
                )
            )
        )
        self.motor.configurator.apply(configuration)

    def get_motor_controller(self) -> phoenix6.hardware.talon_fx.TalonFX:
        return self.motor

    # Getting active voltage
    def get_voltage(self) -> float:
        return self.motor.get_motor_voltage().value

    def disable(self) -> None:
        self.motor.disable()
