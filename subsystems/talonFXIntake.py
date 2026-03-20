import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance
import phoenix6
from phoenix6.controls import Follower
from phoenix6.controls.voltage_out import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import MotorAlignmentValue
from wpilib import SmartDashboard

from hardware.base.motorcontroller import MotorController
from hardware.base.switch import LimitSwitch
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)

# Dumping velocity should be 1500
INTAKE_VOLTAGE = 5.5
DUMP_VOLTAGE = -5.0

EXTENSION_VOLTAGE = 2

MAX_ENCODER_ROTATIONS = 40


class TalonIntakeSubsystem(Subsystem):
    def __init__(
        self,
        intake: MotorController,
        right: TalonFX,
        left: TalonFX,
        forward: LimitSwitch,
        backward: LimitSwitch,
    ) -> None:
        super().__init__()
        self.intake_motor = intake

        intake_config = MotorControllerConfig(
            inverted=False, idle_mode=MotorControllerIdleMode.BRAKE
        )
        self.intake_motor.apply_configs(intake_config)

        self.left = left
        self.right = right

        inverted_value = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        idle_mode = phoenix6.signals.NeutralModeValue.BRAKE

        config = phoenix6.configs.TalonFXConfiguration().with_motor_output(
            phoenix6.configs.MotorOutputConfigs()
            .with_inverted(inverted_value)
            .with_neutral_mode(idle_mode)
        )

        right_config = phoenix6.configs.TalonFXConfiguration().with_motor_output(
            phoenix6.configs.MotorOutputConfigs().with_neutral_mode(idle_mode)
        )

        self.left.configurator.apply(config)
        self.right.configurator.apply(right_config)

        right.set_control(
            Follower(left.device_id, motor_alignment=MotorAlignmentValue.OPPOSED)
        )

        self.left.get_motor_voltage().set_update_frequency(100)

        self.forward = forward
        self.backward = backward

        self.intake_voltage = INTAKE_VOLTAGE
        self.dump_voltage = DUMP_VOLTAGE
        self.extension_voltage = EXTENSION_VOLTAGE

        # setup network tables
        self.nt_inst = NetworkTableInstance.getDefault()

        self.nt_table = self.nt_inst.getTable("intake")

        self.intake_voltage_topic = self.nt_table.getDoubleTopic("intake_motor_voltage")
        self.intake_voltage_pub = self.intake_voltage_topic.publish()
        self.intake_voltage_pub.set(INTAKE_VOLTAGE)
        self.intake_voltage_sub = self.intake_voltage_topic.subscribe(INTAKE_VOLTAGE)

        self.dump_voltage_topic = self.nt_table.getDoubleTopic("dump_motor_voltage")
        self.dump_voltage_pub = self.dump_voltage_topic.publish()
        self.dump_voltage_pub.set(DUMP_VOLTAGE)
        self.dump_voltage_sub = self.dump_voltage_topic.subscribe(DUMP_VOLTAGE)

        self.extension_voltage_topic = self.nt_table.getDoubleTopic(
            "extension_motor_voltage"
        )
        self.extension_voltage_pub = self.extension_voltage_topic.publish()
        self.extension_voltage_pub.set(EXTENSION_VOLTAGE)
        self.extension_voltage_sub = self.extension_voltage_topic.subscribe(
            EXTENSION_VOLTAGE
        )

        self.lock = threading.Lock()

        def _on_intake_voltage_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.intake_voltage = event.data.value.getDouble()
                print(self.intake_voltage)

        self.intake_changed_handle = self.nt_inst.addListener(
            self.intake_voltage_sub,
            ntcore.EventFlags.kValueAll,
            _on_intake_voltage_changed,
        )

        def _on_dump_voltage_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.dump_voltage = event.data.value.getDouble()
                print(self.dump_voltage)

        self.dump_changed_handle = self.nt_inst.addListener(
            self.dump_voltage_sub, ntcore.EventFlags.kValueAll, _on_dump_voltage_changed
        )

        def _on_extension_voltage_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.extension_voltage = event.data.value.getDouble()
                print(self.extension_voltage)

        self.extension_changed_handle = self.nt_inst.addListener(
            self.extension_voltage_sub,
            ntcore.EventFlags.kValueAll,
            _on_extension_voltage_changed,
        )

    def periodic(self) -> None:
        if self.backward_extended():
            self.set_rotations(0)
        elif self.forward_extended():
            self.set_rotations(MAX_ENCODER_ROTATIONS)

        self.nt_table.putBoolean("Forward limit: ", self.forward_extended())
        self.nt_table.putBoolean("Backward limit: ", self.backward_extended())
        self.nt_table.putNumber("Extension encoder", self.get_extension_position())
        SmartDashboard.putBoolean("Intake Active", self.intake_motor.get_voltage() != 0)

    def set_intake_voltage_from_networktable(self) -> None:
        self.intake_motor.set_voltage(self.intake_voltage)

    def set_dump_voltage_from_networktable(self) -> None:
        self.intake_motor.set_voltage(self.dump_voltage)

    def set_intake_voltage(self, voltage: float) -> None:
        self.intake_motor.set_voltage(voltage)

    def set_intake_velocity(self, rpm: float) -> None:
        self.intake_motor.set_velocity(rpm)

    def set_extension_voltage(self, voltage: float) -> None:
        if (voltage > 0 and self.forward_extended()) or (
            voltage < 0 and self.backward_extended()
        ):
            self.left.set_control(VoltageOut(0))
        else:
            self.left.set_control(VoltageOut(voltage))

    def set_extension_voltage_from_networktable(self) -> None:
        if not self.forward_extended():
            self.left.set_control(VoltageOut(self.extension_voltage))
        else:
            self.left.set_control(VoltageOut(0))

    def set_retraction_voltage_from_networktable(self) -> None:
        if not self.backward_extended():
            self.left.set_control(VoltageOut(-self.extension_voltage))
        else:
            self.left.set_control(VoltageOut(0))

    def forward_extended(self) -> bool:
        return self.forward.get_state()

    def backward_extended(self) -> bool:
        return self.backward.get_state()

    def set_rotations(self, rotations: float = 0) -> None:
        self.left.set_position(rotations)

    def get_extension_position(self) -> float:
        return self.left.get_position().value

    def get_max_extension_value(self) -> float:
        return MAX_ENCODER_ROTATIONS
