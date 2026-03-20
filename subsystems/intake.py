import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance

from hardware.base.motorcontroller import MotorController
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)

# Dumping velocity should be 1500
INTAKE_VOLTAGE = 5.5
DUMP_VOLTAGE = -5.0


class IntakeSubsystem(Subsystem):
    def __init__(
        self,
        intake: MotorController,
    ) -> None:
        super().__init__()
        self.intake_motor = intake

        intake_config = MotorControllerConfig(
            inverted=False, idle_mode=MotorControllerIdleMode.BRAKE
        )
        self.intake_motor.apply_configs(intake_config)

        self.intake_voltage = INTAKE_VOLTAGE
        self.dump_voltage = DUMP_VOLTAGE

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

    def periodic(self) -> None:
        pass

    def set_intake_voltage_from_networktable(self) -> None:
        self.intake_motor.set_voltage(self.intake_voltage)

    def set_dump_voltage_from_networktable(self) -> None:
        self.intake_motor.set_voltage(self.dump_voltage)

    def set_intake_voltage(self, voltage: float) -> None:
        self.intake_motor.set_voltage(voltage)

    def set_intake_velocity(self, rpm: float) -> None:
        self.intake_motor.set_velocity(rpm)
