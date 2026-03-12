import commands2.subsystem
import ntcore

from hardware.base.motorcontroller import MotorController

CLIMB_VOLTAGE = 5


class ClimberSubsystem(commands2.Subsystem):
    def __init__(self, climb_motor: MotorController):
        super().__init__()
        self.climb_motor = climb_motor

        nt_inst = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = nt_inst.getTable("climber")
        self.climber_voltage_topic = self.nt_table.getDoubleTopic(
            "climber_motor_voltage"
        )
        self.climber_voltage_pub = self.climber_voltage_topic.publish()
        self.climber_voltage_pub.set(CLIMB_VOLTAGE)
        self.climber_voltage_sub = self.climber_voltage_topic.subscribe(CLIMB_VOLTAGE)

    def motor_extend(self) -> None:
        voltage = self.climber_voltage_sub.get()
        self.climb_motor.set_voltage(voltage)

    def motor_retract(self) -> None:
        voltage = self.climber_voltage_sub.get()
        self.climb_motor.set_voltage(-voltage)

    def motor_stop(self) -> None:
        self.climb_motor.set_voltage(0)
