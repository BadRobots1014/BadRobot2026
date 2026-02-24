import phoenix6
from phoenix6.controls.velocity_voltage import VelocityVoltage
from phoenix6.controls.voltage_out import VoltageOut
from phoenix6.units import rotations_per_second

from hardware.base.encoder import Encoder
from hardware.base.motor import Motor


class Kraken(Motor):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = phoenix6.hardware.talon_fx.TalonFX(motor_id)
        self.motor_id = motor_id

    def set_voltage(self, voltage: float):
        self.motor.set_control(VoltageOut(voltage))

    def set_velocity(self, velocity: float) -> None:
        self.motor.set_control(VelocityVoltage(rotations_per_second(velocity)))
        return None

    def set_inverted(self, inverted: bool):
        configuration = phoenix6.configs.TalonFXConfiguration().with_motor_output(
            phoenix6.configs.MotorOutputConfigs().with_inverted(
                phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
            )
        )
        self.motor.configurator.apply(configuration)

    def set_leader(self, leader: int, oppose: bool) -> None:
        self.motor.set_control(
            phoenix6.controls.follower.Follower(
                leader,
                motor_alignment=(
                    phoenix6.signals.MotorAlignmentValue.OPPOSED
                    if oppose
                    else phoenix6.signals.MotorAlignmentValue.ALIGNED
                ),
            )
        )

    def get_motor_controller(self) -> phoenix6.hardware.talon_fx.TalonFX:
        return self.motor

    # Getting active voltage
    def get_voltage(self) -> float:
        return self.motor.get_motor_voltage().value

    def get_encoder(self) -> Encoder:
        raise Exception("Not Implemented")

    def get_motor_id(self) -> int:
        return self.motor_id

    def get_forward_limit(self) -> bool:
        raise Exception("Not Implemented")

    def get_backward_limit(self) -> bool:
        raise Exception("Not Implemented")

    def disable(self) -> None:
        self.motor.set_control(VoltageOut(0))
