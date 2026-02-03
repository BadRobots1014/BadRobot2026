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
        use_pid: bool = False,
    ):
        self.shoot_motor = shoot_motor
        self.turn_motor = turn_motor

        self.shoot_encoder = shoot_encoder
        self.turn_encoder = turn_encoder

        self.shoot_velocity = 0

        self.use_pid = use_pid

    def set_shoot_voltage(self, volts: float):
        self.shoot_motor.set_voltage(volts)

    def set_shoot_velocity(self, velocity: float):
        self.shoot_velocity = velocity

    def set_turn_voltage(self, volts: float):
        self.turn_motor.set_voltage(volts)

    def reset_shoot(self):
        self.shoot_encoder.set_position(0)

    def reset_turn(self):
        self.turn_encoder.set_position(0)

    def shoot_pid_update(self, volts: float):
        if not self.use_pid:
            return
        self.shoot_motor.set_voltage(volts)

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
