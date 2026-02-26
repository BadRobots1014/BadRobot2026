import rev

from hardware.base.encoder import Encoder
from hardware.base.motorcontroller import MotorController
from hardware.impl.spark_relative_encoder import SparkRelativeEncoder
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)


class SparkFlexMotorController(MotorController):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = rev.SparkFlex(motor_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.controller = self.motor.getClosedLoopController()

    def set_voltage(self, voltage: float):
        self.motor.setVoltage(voltage)

    def set_inverted(self, inverted: bool):
        self.motor.setInverted(inverted)

    def set_velocity(self, velocity: float) -> None:
        self.controller.setSetpoint(
            velocity, rev._rev.SparkLowLevel.ControlType.kVelocity
        )

    def get_encoder(self) -> Encoder:
        return SparkRelativeEncoder(self.motor.getEncoder())

    def get_motor_controller(self) -> rev.SparkBase:
        return self.motor

    def get_motor_id(self) -> int:
        return self.motor.getDeviceId()

    # Getting active voltage
    def get_voltage(self) -> float:
        return self.motor.getBusVoltage() * self.motor.getAppliedOutput()

    def get_forward_limit(self) -> bool:
        return self.motor.getForwardLimitSwitch().get()

    def get_backward_limit(self) -> bool:
        return self.motor.getReverseLimitSwitch().get()

    def apply_configs(self, motor_controller_config: MotorControllerConfig) -> None:
        config = rev.SparkFlexConfig()
        config.inverted(motor_controller_config.inverted)

        idleMode = (
            rev.SparkFlexConfig.IdleMode.kBrake
            if motor_controller_config.idle_mode == MotorControllerIdleMode.BRAKE
            else rev.SparkFlexConfig.IdleMode.kCoast
        )

        config.IdleMode(idleMode)
        if motor_controller_config.leader is not None:
            config.follow(motor_controller_config.leader.get_motor_controller())

        self.motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def disable(self) -> None:
        self.motor.disable()
