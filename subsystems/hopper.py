import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance
import phoenix6
from phoenix6 import configs
from phoenix6.controls import Follower, PositionVoltage
from phoenix6.controls.voltage_out import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import MotorAlignmentValue

from hardware.base.switch import LimitSwitch

EXTENSION_VOLTAGE = 3

MAX_ENCODER_ROTATIONS = 10


class HopperSubsystem(Subsystem):
    def __init__(
        self,
        left_motor: TalonFX,
        right_motor: TalonFX,
        forward_limit_switch: LimitSwitch,
    ):
        super().__init__()

        self.left_motor = left_motor
        self.right_motor = right_motor

        counter_clockwise_positive = (
            phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        clockwise_positive = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        brake = phoenix6.signals.NeutralModeValue.BRAKE
        coast = phoenix6.signals.NeutralModeValue.COAST

        # in init function, set slot 0 gains
        slot0_configs = configs.Slot0Configs()
        slot0_configs.k_s = 0.25  # Add 0.25 V output to overcome static friction
        slot0_configs.k_v = 0.12  # A velocity target of 1 rps results in 0.12 V output
        slot0_configs.k_p = (
            4.8  # A position error of 2.5 rotations results in 12 V output
        )
        slot0_configs.k_i = 0  # no output for integrated error
        slot0_configs.k_d = 0.1  # A velocity error of 1 rps results in 0.1 V output

        self.leader_brake_config = (
            phoenix6.configs.TalonFXConfiguration()
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs()
                .with_inverted(counter_clockwise_positive)
                .with_neutral_mode(brake)
            )
            .with_slot0(slot0_configs)
        )

        self.follower_brake_config = (
            phoenix6.configs.TalonFXConfiguration()
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs()
                .with_inverted(clockwise_positive)
                .with_neutral_mode(brake)
            )
            .with_slot0(slot0_configs)
        )

        self.leader_coast_config = (
            phoenix6.configs.TalonFXConfiguration()
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs()
                .with_inverted(counter_clockwise_positive)
                .with_neutral_mode(coast)
            )
            .with_slot0(slot0_configs)
        )

        self.follower_coast_config = (
            phoenix6.configs.TalonFXConfiguration()
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs()
                .with_inverted(clockwise_positive)
                .with_neutral_mode(coast)
            )
            .with_slot0(slot0_configs)
        )

        self.left_motor.configurator.apply(self.leader_brake_config)
        self.right_motor.configurator.apply(self.follower_brake_config)

        self.right_motor.set_control(
            Follower(
                self.left_motor.device_id, motor_alignment=MotorAlignmentValue.OPPOSED
            )
        )

        self.left_motor.get_motor_voltage().set_update_frequency(100)

        self.forward_limit_switch = forward_limit_switch

        self.extension_voltage = EXTENSION_VOLTAGE

        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable("intake")

        self.lock = threading.Lock()

        self.extension_voltage_topic = self.nt_table.getDoubleTopic(
            "extension_motor_voltage"
        )
        self.extension_voltage_pub = self.extension_voltage_topic.publish()
        self.extension_voltage_pub.set(EXTENSION_VOLTAGE)
        self.extension_voltage_sub = self.extension_voltage_topic.subscribe(
            EXTENSION_VOLTAGE
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
        if self.forward_extended():
            self.set_rotations(MAX_ENCODER_ROTATIONS)

        self.nt_table.putBoolean("Forward limit: ", self.forward_extended())
        self.nt_table.putNumber("Extension encoder", self.get_extension_position())

    def set_extension_voltage(self, voltage: float) -> None:
        if (self.forward_extended()) or (voltage < 0 and self.backward_extended()):
            self.left_motor.set_control(VoltageOut(0))
        else:
            self.left_motor.set_control(VoltageOut(voltage))

    def set_extension_voltage_from_networktable(self) -> None:
        if not self.forward_extended():
            self.left_motor.set_control(VoltageOut(-self.extension_voltage))
        else:
            self.left_motor.set_control(VoltageOut(0))

    def set_retraction_voltage_from_networktable(self) -> None:
        if not self.backward_extended() and not self.forward_extended():
            self.left_motor.set_control(VoltageOut(self.extension_voltage))
        else:
            self.left_motor.set_control(VoltageOut(0))

    def set_extension_position_and_velocity(self, request: PositionVoltage) -> None:
        self.left_motor.set_control(request)

    def forward_extended(self) -> bool:
        return self.forward_limit_switch.get_state()

    def set_rotations(self, rotations: float = 0) -> None:
        self.left_motor.set_position(rotations)

    def get_extension_position(self) -> float:
        return self.left_motor.get_position().value

    def get_max_extension_value(self) -> float:
        return MAX_ENCODER_ROTATIONS

    def set_coast(self) -> None:
        self.left_motor.configurator.apply(self.leader_coast_config)
        self.right_motor.configurator.apply(self.follower_coast_config)
        print("applied coast to hopper")

    def set_brake(self) -> None:
        self.left_motor.configurator.apply(self.leader_brake_config)
        self.right_motor.configurator.apply(self.follower_brake_config)
        print("applied brake to hopper")
