import phoenix6

from phoenix6.controls.velocity_voltage import VelocityVoltage
from phoenix6.controls.voltage_out import VoltageOut
from phoenix6.units import rotations_per_second

from hardware.base.encoder import Encoder
from hardware.base.motorcontroller import MotorController
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)


class TalonFXMotorController(MotorController):
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
        invertedValue = (
            phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
            if inverted
            else phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        configuration = phoenix6.configs.TalonFXConfiguration().with_motor_output(
            phoenix6.configs.MotorOutputConfigs().with_inverted(invertedValue)
        )
        self.motor.configurator.apply(configuration)

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

    def get_inverted(self) -> bool:
        config = phoenix6.configs.MotorOutputConfigs()
        self.get_motor_controller().configurator.refresh(config)
        return (
            config.inverted == phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

    def apply_configs(self, motor_controller_config: MotorControllerConfig) -> None:
        invertedValue = (
            phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
            if motor_controller_config.inverted
            else phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        idleMode = (
            phoenix6.signals.NeutralModeValue.BRAKE
            if motor_controller_config.idle_mode == MotorControllerIdleMode.BRAKE
            else phoenix6.signals.NeutralModeValue.COAST
        )

        config = phoenix6.configs.TalonFXConfiguration().with_motor_output(
            phoenix6.configs.MotorOutputConfigs()
            .with_inverted(invertedValue)
            .with_neutral_mode(idleMode)
        )

        config.slot0.k_p = motor_controller_config.pidf[0]
        config.slot0.k_i = motor_controller_config.pidf[1]
        config.slot0.k_d = motor_controller_config.pidf[2]
        config.slot0.k_v = motor_controller_config.pidf[3]

        self.motor.configurator.apply(config)

        motor_controller_config.leader

        if (
            motor_controller_config.leader is not None
            and motor_controller_config.leader is self.__class__
        ):

            self.motor.set_control(
                phoenix6.controls.follower.Follower(
                    motor_controller_config.leader.get_motor_id(),
                    motor_alignment=(
                        phoenix6.signals.MotorAlignmentValue.OPPOSED
                        if motor_controller_config.inverted ^ self.get_inverted()
                        else phoenix6.signals.MotorAlignmentValue.ALIGNED
                    ),
                )
            )

    def disable(self) -> None:
        self.motor.set_control(VoltageOut(0))
