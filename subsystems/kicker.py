import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance
import wpilib

from hardware.base.encoder import Encoder
from hardware.base.motorcontroller import MotorController
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)

UNJAM_SPIN_TIME = 1  # time to spin to unjam in seconds
JAM_TIME = 1  # time to be considered jammed in seconds
JAM_RPM = 50  # rpm threshold to be considered jammed

KICKER_SHOOT_VOLTAGE = 4.0
KICKER_DUMP_VOLTAGE = 2.0

KICKER_P = 0.001
KICKER_I = 0
KICKER_D = 0
KICKER_F = 0.00181111111  # trusting dre


class KickerSubsystem(Subsystem):
    def __init__(
        self,
        kick_motor: MotorController,
        kick_encoder: Encoder,
    ):
        super().__init__()

        self.kick_motor = kick_motor
        self.kick_encoder = kick_encoder

        self.kick_shoot_voltage = KICKER_SHOOT_VOLTAGE
        self.kick_dump_voltage = KICKER_DUMP_VOLTAGE

        # Config kick motor
        kick_config = MotorControllerConfig(
            inverted=False,
            idle_mode=MotorControllerIdleMode.BRAKE,
            p=KICKER_P,
            i=KICKER_I,
            d=KICKER_D,
            f=KICKER_F,
        )
        self.kick_motor.apply_configs(kick_config)

        self._inst = NetworkTableInstance.getDefault()
        self._shooter_table = self._inst.getTable("ShooterTable")
        # Create nt topics
        self._kicker_shoot_motor_voltage_topic = self._shooter_table.getDoubleTopic(
            "KickerShooterMotorVoltage"
        )
        self._kicker_dump_motor_voltage_topic = self._shooter_table.getDoubleTopic(
            "KickerDumpMotorVoltage"
        )

        # set nt defaults
        self._kicker_shoot_motor_voltage_pub = (
            self._kicker_shoot_motor_voltage_topic.publish()
        )
        self._kicker_shoot_motor_voltage_pub.set(self.kick_shoot_voltage)
        self._kicker_dump_motor_voltage_pub = (
            self._kicker_dump_motor_voltage_topic.publish()
        )
        self._kicker_dump_motor_voltage_pub.set(self.kick_dump_voltage)

        # create nt subscribers
        self._kicker_shoot_motor_voltage_sub = self._kicker_shoot_motor_voltage_topic.subscribe(
            0  # default value so we know something is going wrong with network tables
        )
        self._kicker_dump_motor_voltage_sub = self._kicker_dump_motor_voltage_topic.subscribe(
            0  # default value so we know something is going wrong with network tables
        )

        # set up listeners

        self.lock = threading.Lock()

        def _on_kicker_shoot_voltage_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.kick_shoot_voltage = event.data.value.getDouble()
                print(self.kick_shoot_voltage)

        self.kickerShootListenerHandle = self._inst.addListener(
            self._kicker_shoot_motor_voltage_sub,
            ntcore.EventFlags.kValueAll,
            _on_kicker_shoot_voltage_changed,
        )

        def _on_kicker_dump_voltage_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.kick_dump_voltage = event.data.value.getDouble()
                print(self.kick_dump_voltage)

        self.kickerDumpListenerHandle = self._inst.addListener(
            self._kicker_dump_motor_voltage_sub,
            ntcore.EventFlags.kValueAll,
            _on_kicker_dump_voltage_changed,
        )

    def set_kick_voltage(self, volts: float) -> None:
        self.kick_motor.set_voltage(volts)

    def set_kick_velocity(self, velocity: float) -> None:
        self.kick_motor.set_velocity(velocity)

    def set_kick_shoot_voltage_from_networktables(self) -> None:
        self.set_kick_voltage(self.kick_shoot_voltage)

    def set_kick_dump_voltage_from_networktables(self) -> None:
        self.set_kick_voltage(self.kick_dump_voltage)

    def reset_kick(self) -> None:
        self.kick_encoder.set_position(0)

    def kick_unjam(self) -> None:
        # first if checks for first instance of jamming
        if self.time_of_stall == -1 and self.kick_encoder.get_velocity() < JAM_RPM:
            self.time_of_stall = wpilib.RobotController.getFPGATime()
            return
        # gets current time jammed
        time_stalled = wpilib.RobotController.getFPGATime() - self.time_of_stall
        # check if jammed for more than once second
        if (
            self.time_of_stall != -1
            and self.kick_encoder.get_velocity() < JAM_RPM
            and time_stalled > JAM_TIME
        ):
            # start unjam process and track time
            self.start_unjam = wpilib.RobotController.getFPGATime()
            self.kick_motor.set_velocity(-self.kick_shoot_voltage)
            return
        time_unjamming = wpilib.RobotController.getFPGATime() - self.start_unjam
        # go normal if unjamming for more than one second
        if time_unjamming > UNJAM_SPIN_TIME:
            self.kick_motor.set_velocity(self.kick_shoot_voltage)
            return
        return

    def periodic(self) -> None:
        # constantly checks procedure for unjam
        # self.kick_unjam()
        return

    @property
    def kick_distance(self) -> float:
        return self.kick_encoder.get_position()