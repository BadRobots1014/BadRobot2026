import rev

from hardware.base.encoder import Encoder
from hardware.base.motorcontroller import MotorController
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)
from hardware.impl.spark_relative_encoder import SparkRelativeEncoder


class SparkFlexMotorController(MotorController):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = rev.SparkFlex(motor_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.controller = self.motor.getClosedLoopController()

    def set_voltage(self, voltage: float) -> None:
        self.motor.setVoltage(voltage)

    def set_inverted(self, inverted: bool) -> None:
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

        idle_mode = (
            rev.SparkFlexConfig.IdleMode.kBrake
            if motor_controller_config.idle_mode == MotorControllerIdleMode.BRAKE
            else rev.SparkFlexConfig.IdleMode.kCoast
        )

        config.IdleMode(idle_mode)
        if motor_controller_config.leader is not None:
            leader = motor_controller_config.leader.get_motor_controller()
            if isinstance(leader, rev.SparkBase):
                config.follow(leader, motor_controller_config.inverted)
            else:
                raise TypeError(
                    f"SparkFlex cannot follow a non-Spark leader: {type(leader)}"
                )

        pid_config = rev.ClosedLoopConfig()
        pid_config.pidf(
            motor_controller_config.p,
            motor_controller_config.i,
            motor_controller_config.d,
            motor_controller_config.f,
        )
        config.apply(pid_config)

        self.motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def disable(self) -> None:
        self.motor.disable()
