import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance
import phoenix6
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX

CONVEYOR_VOLTAGE = 4


class Conveyor(Subsystem):
    def __init__(self, conveyor_motor: TalonFX):
        super().__init__()
        self.conveyor_motor = conveyor_motor

        clockwise_positive = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        idle_mode = phoenix6.signals.NeutralModeValue.BRAKE

        conveyor_config = phoenix6.configs.TalonFXConfiguration().with_motor_output(
            phoenix6.configs.MotorOutputConfigs()
            .with_inverted(clockwise_positive)
            .with_neutral_mode(idle_mode)
        )

        self.conveyor_motor.configurator.apply(conveyor_config)

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
        self.conveyor_motor.set_control(VoltageOut(self.conveyor_voltage))

    def set_conveyor_dump_voltage_from_networktable(self) -> None:
        self.conveyor_motor.set_control(VoltageOut(-self.conveyor_voltage))
