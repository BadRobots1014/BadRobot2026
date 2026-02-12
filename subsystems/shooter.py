from hardware.base.encoder import Encoder
from hardware.base.motor import Motor
from hardware.impl.spark_flex_motor import SparkFlexMotor


class Shooter:
    # def __init__(self, shoot_motor_id: int, turn_motor_id: int):
    def __init__(self):
        self.shoot_motor = SparkFlexMotor(0)
        self.kick_motor = SparkFlexMotor(0)

        self.shoot_encoder = self.shoot_motor.get_encoder()
        self.kick_encoder = self.kick_motor.get_encoder()

        self.shoot_velocity = 0

    def set_shoot_voltage(self, volts: float):
        self.shoot_motor.set_voltage(volts)

    def set_shoot_velocity(self, velocity: float):
        self.shoot_velocity = velocity
        self.shoot_motor.set_velocity(velocity)

    def set_kick_voltage(self, volts: float):
        self.kick_motor.set_voltage(volts)

    def reset_shoot(self):
        self.shoot_encoder.set_position(0)

    def reset_kick(self):
        self.kick_encoder.set_position(0)

    @property
    def shoot_distance(self) -> float:
        return self.shoot_encoder.get_position()

    @property
    def kick_distance(self) -> float:
        return self.kick_encoder.get_position()

    @property
    def shoot_voltage(self) -> float:
        return self.shoot_motor.get_voltage()

    @property
    def kick_voltage(self) -> float:
        return self.kick_motor.get_voltage()
