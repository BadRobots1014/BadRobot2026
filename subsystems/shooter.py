import threading

import ntcore
import wpilib
from commands2 import Subsystem
from ntcore import NetworkTableInstance

from hardware.base.encoder import Encoder
from hardware.base.motorcontroller import MotorController
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)

UNJAM_SPIN_TIME = 1  # time to spin to unjam in seconds
JAM_TIME = 1  # time to be considered jammed in seconds
JAM_RPM = 50  # rpm threshold to be considered jammed

SHOOTER_VELOCITY = 4500
KICKER_VOLTAGE = 600

SHOOTER_P = .001
SHOOTER_I = 0
SHOOTER_D = 0
SHOOTER_F = .00181111111 # trusting dre

class ShooterSubsystem(Subsystem):
    def __init__(
        self,
        main_shoot_motor: MotorController,
        follower_shoot_motor: MotorController,
        shoot_encoder: Encoder,
        kick_motor: MotorController,
        kick_encoder: Encoder,
    ):
        super().__init__()

        self.shoot_motor = main_shoot_motor
        self.f_shoot_motor = follower_shoot_motor

        self.kick_motor = kick_motor

        self.shoot_encoder = shoot_encoder
        self.kick_encoder = kick_encoder

        self.shoot_velocity = SHOOTER_VELOCITY
        self.kick_voltage = KICKER_VOLTAGE

        # tracks time for automatic jamming procedures
        self.time_of_stall = -1
        self.start_unjam = -1

        # Config shoot motor
        self.shoot_config = MotorControllerConfig(
            inverted=False,
            idle_mode=MotorControllerIdleMode.COAST,
            pidf=[SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F],
        )
        self.shoot_motor.apply_configs(self.shoot_config)

        # Config kick motor
        kick_config = MotorControllerConfig(
            inverted=False, idle_mode=MotorControllerIdleMode.BRAKE
        )
        self.kick_motor.apply_configs(kick_config)

        # Config follower motor
        follower_config = MotorControllerConfig(
            inverted=True,
            idle_mode=MotorControllerIdleMode.COAST,
            pidf=[SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F],
            leader=self.shoot_motor,
        )
        self.f_shoot_motor.apply_configs(follower_config)

        self._inst = NetworkTableInstance.getDefault()
        self._shooter_table = self._inst.getTable("ShooterTable")
        # Create nt topics
        self._shooter_motor_velocity_topic = self._shooter_table.getDoubleTopic(
            "ShooterMotorVelocity"
        )
        self._kicker_motor_voltage_topic = self._shooter_table.getDoubleTopic(
            "KickerMotorVoltage"
        )

        # set nt defaults
        self._shooter_motor_velocity_pub = self._shooter_motor_velocity_topic.publish()
        self._shooter_motor_velocity_pub.set(self.shoot_velocity)
        self._kicker_motor_voltage_pub = self._kicker_motor_voltage_topic.publish()
        self._kicker_motor_voltage_pub.set(self.kick_voltage)

        # create nt subscribers
        self._shooter_motor_velocity_sub = self._shooter_motor_velocity_topic.subscribe(
            0  # default value so we know something is going wrong with network tables
        )
        self._kicker_motor_voltage_sub = self._kicker_motor_voltage_topic.subscribe(
            0  # default value so we know something is going wrong with network tables
        )

        self._shooter_p_topic = self._shooter_table.getDoubleTopic("Shooter P")
        self._shooter_i_topic = self._shooter_table.getDoubleTopic("Shooter I")
        self._shooter_d_topic = self._shooter_table.getDoubleTopic("Shooter D")
        self._shooter_f_topic = self._shooter_table.getDoubleTopic("Shooter F")

        self._shooter_p_pub = self._shooter_p_topic.publish()
        self._shooter_p_pub.set(SHOOTER_P)
        self._shooter_i_pub = self._shooter_i_topic.publish()
        self._shooter_i_pub.set(SHOOTER_I)
        self._shooter_d_pub = self._shooter_d_topic.publish()
        self._shooter_d_pub.set(SHOOTER_D)
        self._shooter_f_pub = self._shooter_f_topic.publish()
        self._shooter_f_pub.set(SHOOTER_F)

        self._shooter_p_sub = self._shooter_p_topic.subscribe(SHOOTER_P)
        self._shooter_i_sub = self._shooter_i_topic.subscribe(SHOOTER_I)
        self._shooter_d_sub = self._shooter_d_topic.subscribe(SHOOTER_D)
        self._shooter_f_sub = self._shooter_f_topic.subscribe(SHOOTER_F)

        # set up listeners

        self.lock = threading.Lock()

        def _on_shooter_rpm_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_velocity = event.data.value.getDouble()
                print(self.shoot_velocity)

        self.shooterListenerHandle = self._inst.addListener(
            self._shooter_motor_velocity_sub, ntcore.EventFlags.kValueAll, _on_shooter_rpm_changed
        )


        def _on_kicker_voltage_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.kick_voltage = event.data.value.getDouble()
                print(self.kick_voltage)

        self.kickerListenerHandle = self._inst.addListener(
            self._kicker_motor_voltage_sub, ntcore.EventFlags.kValueAll, _on_kicker_voltage_changed
        )

        def _on_shooter_p_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.pidf[0] = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_p_changed_handle = self._inst.addListener(
            self._shooter_p_sub, ntcore.EventFlags.kValueAll, _on_shooter_p_changed
        )

        def _on_shooter_i_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.pidf[1] = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_i_changed_handle = self._inst.addListener(
            self._shooter_i_sub, ntcore.EventFlags.kValueAll, _on_shooter_i_changed
        )

        def _on_shooter_d_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.pidf[2] = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_d_changed_handle = self._inst.addListener(
            self._shooter_d_sub, ntcore.EventFlags.kValueAll, _on_shooter_d_changed
        )

        def _on_shooter_f_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.pidf[3] = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_f_changed_handle = self._inst.addListener(
            self._shooter_f_sub, ntcore.EventFlags.kValueAll, _on_shooter_f_changed
        )

    def set_shoot_voltage(self, volts: float) -> None:
        self.shoot_motor.set_voltage(volts)

    def set_shoot_velocity(self, velocity: float) -> None:
        self.shoot_motor.set_velocity(velocity)

    def set_shoot_velocity_from_networktables(self) -> None:
        self.set_shoot_velocity(self.shoot_velocity)

    def set_kick_voltage(self, volts: float) -> None:
        self.kick_motor.set_voltage(volts)

    def set_kick_velocity(self, velocity: float) -> None:
        self.kick_motor.set_velocity(velocity)

    def set_kick_velocity_from_networktables(self) -> None:
        self.set_kick_velocity(self.kick_voltage)

    def reset_shoot(self) -> None:
        self.shoot_encoder.set_position(0)

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
            self.kick_motor.set_velocity(-self.kick_voltage)
            return
        time_unjamming = wpilib.RobotController.getFPGATime() - self.start_unjam
        # go normal if unjamming for more than one second
        if time_unjamming > UNJAM_SPIN_TIME:
            self.kick_motor.set_velocity(self.kick_voltage)
            return
        return

    def periodic(self) -> None:
        # constantly checks procedure for unjam
        # self.kick_unjam()
        return

    @property
    def shoot_distance(self) -> float:
        return self.shoot_encoder.get_position()

    @property
    def kick_distance(self) -> float:
        return self.kick_encoder.get_position()