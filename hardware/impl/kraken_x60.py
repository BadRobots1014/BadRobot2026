from hardware.base.motor import Motor
import phoenix6


class Kraken(Motor):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = phoenix6.hardware.talon_fx.TalonFX(motor_id)
        self.motor_id = motor_id

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

    def set_leader(self, leader: int) -> None:
        self.motor.set_control(phoenix6.controls.follower.Follower(leader, phoenix6.signals.MotorAlignmentValue.OPPOSED))

    def get_motor_controller(self) -> phoenix6.hardware.talon_fx.TalonFX:
        return self.motor

    # Getting active voltage
    def get_voltage(self) -> float:
        return self.motor.get_motor_voltage().value

    def get_motor_id(self) -> int:
        return self.motor_id

    def disable(self) -> None:
        self.motor.disable()
