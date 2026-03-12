import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance

from hardware.base.motorcontroller import MotorController
from hardware.base.switch import LimitSwitch
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)

# Dumping velocity should be 1500
INTAKE_VOLTAGE = 4.5
DUMP_VOLTAGE = -5

EXTENSION_VOLTAGE = 3

ENCODER_ROTATIONS = 0
ROTATIONS_TO_METERS = (
    0  # TODO measure how much the hopper extends for one encoder rotation
)


class IntakeSubsystem(Subsystem):
    def __init__(
        self,
        intake: MotorController,
        left: MotorController,
        right: MotorController,
        forward: LimitSwitch,
        backward: LimitSwitch,
        camera_name: str = "limelight",
    ) -> None:
        super().__init__()
        self.intake_motor = intake

        intake_config = MotorControllerConfig(
            inverted=False, idle_mode=MotorControllerIdleMode.BRAKE
        )
        self.intake_motor.apply_configs(intake_config)

        self.left = left
        self.right = right

        left_config = MotorControllerConfig(
            inverted=False, idle_mode=MotorControllerIdleMode.BRAKE
        )
        right_config = MotorControllerConfig(
            inverted=True, idle_mode=MotorControllerIdleMode.BRAKE, leader=self.left
        )

        self.left.apply_configs(left_config)
        self.right.apply_configs(right_config)

        self.forward = forward
        self.backward = backward

        self.intake_voltage = INTAKE_VOLTAGE
        self.dump_voltage = DUMP_VOLTAGE
        self.extension_voltage = EXTENSION_VOLTAGE

        # setup network tables
        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_ll_table = self.nt_inst.getTable(camera_name)
        self.pose_publisher = self.nt_ll_table.getDoubleArrayTopic(
            "camerapose_robotspace"
        ).publish()
        self.pos_subscriber = self.nt_ll_table.getDoubleArrayTopic(
            "camerapose_robotspace"
        ).subscribe([0, 0, 0, 0, 0, 0])

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
            self.zero_rotations()

        if (self.forward_extended() and self.left.get_voltage() < 0) or (
            self.backward_extended() and self.left.get_voltage() > 0
        ):
            self.set_extension_voltage(0)

    def set_intake_voltage_from_networktable(self) -> None:
        self.intake_motor.set_voltage(self.intake_voltage)

    def set_dump_voltage_from_networktable(self) -> None:
        self.intake_motor.set_voltage(self.dump_voltage)

    def set_intake_voltage(self, voltage: float) -> None:
        self.intake_motor.set_voltage(voltage)

    def set_intake_velocity(self, rpm: float) -> None:
        self.intake_motor.set_velocity(rpm)

    def set_extension_voltage(self, voltage: float) -> None:
        self.left.set_voltage(voltage)

    def set_extention_voltage_from_networktable(self) -> None:
        self.left.set_voltage(self.extension_voltage)

    def forward_extended(self) -> bool:
        return self.forward.get_state()

    def backward_extended(self) -> bool:
        return self.backward.get_state()

    def zero_rotations(self) -> None:
        self.left.zero_relative_encoder()
        self.right.zero_relative_encoder()

    def get_extension_position(self) -> float:
        return self.left.get_encoder_position() * ROTATIONS_TO_METERS
