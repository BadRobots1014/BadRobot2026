import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance

from hardware.impl.motor_controller_config import MotorControllerConfig
from hardware.impl.spark_flex_motor import SparkFlexMotorController

CONVEYOR_VOLTAGE = 10


class ConveyorSubsystem(Subsystem):
    def __init__(self, conveyor_motor: SparkFlexMotorController):
        super().__init__()
        self.conveyor_motor = conveyor_motor

        conveyor_config = MotorControllerConfig()
        self.conveyor_motor.apply_configs(conveyor_config)

        self.conveyor_voltage = CONVEYOR_VOLTAGE
        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable("conveyor")
        self.lock = threading.Lock()

        self.conveyor_voltage_topic = self.nt_table.getDoubleTopic(
            "conveyor_motor_voltage"
        )
        self.conveyor_voltage_pub = self.conveyor_voltage_topic.publish()
        self.conveyor_voltage_pub.set(CONVEYOR_VOLTAGE)
        self.conveyor_voltage_sub = self.conveyor_voltage_topic.subscribe(
            CONVEYOR_VOLTAGE,
        )

        def _on_conveyor_voltage_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.conveyor_voltage = event.data.value.getDouble()
                print(self.conveyor_voltage)

        self.extension_changed_handle = self.nt_inst.addListener(
            self.conveyor_voltage_sub,
            ntcore.EventFlags.kValueAll,
            _on_conveyor_voltage_changed,
        )

    def set_conveyor_shoot_voltage_from_networktable(self) -> None:
        self.conveyor_motor.set_voltage(self.conveyor_voltage)

    def set_conveyor_dump_voltage_from_networktable(self) -> None:
        self.conveyor_motor.set_voltage(-self.conveyor_voltage)

    def set_conveyor_voltage(self, voltage: float) -> None:
        self.conveyor_motor.set_voltage(voltage)
