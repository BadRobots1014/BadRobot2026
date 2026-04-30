import threading

from commands2 import Subsystem
import ntcore
from ntcore import NetworkTableInstance

from hardware.base.encoder import Encoder
from hardware.base.motorcontroller import MotorController
from hardware.impl.motor_controller_config import (
    MotorControllerConfig,
    MotorControllerIdleMode,
)
import robot

SHOOTER_VELOCITY = 4500

SHOOTER_P = 0.001
SHOOTER_I = 0
SHOOTER_D = 0
SHOOTER_F = 0.00181111111  # trusting dre

# radius: meters, shooter speed: rpm
SHOOT_PAIRS = [(3.8128, 3200)]


class ShooterSubsystem(Subsystem):
    def __init__(
        self,
        main_shoot_motor: MotorController,
        follower_shoot_motor: MotorController,
        shoot_encoder: Encoder,
    ):
        super().__init__()

        self.shoot_motor = main_shoot_motor
        self.f_shoot_motor = follower_shoot_motor

        self.shoot_encoder = shoot_encoder

        self.target_velocity = SHOOTER_VELOCITY

        # tracks time for automatic jamming procedures
        self.time_of_stall = -1
        self.start_unjam = -1

        self.closest_pair = (0, 0)

        # Config shoot motor
        self.shoot_config = MotorControllerConfig(
            inverted=False,
            idle_mode=MotorControllerIdleMode.COAST,
            p=SHOOTER_P,
            i=SHOOTER_I,
            d=SHOOTER_D,
            f=SHOOTER_F,
        )
        self.shoot_motor.apply_configs(self.shoot_config)

        # Config follower motor
        follower_config = MotorControllerConfig(
            inverted=True,
            idle_mode=MotorControllerIdleMode.COAST,
            leader=self.shoot_motor,
        )
        self.f_shoot_motor.apply_configs(follower_config)

        self._inst = NetworkTableInstance.getDefault()
        self._shooter_table = self._inst.getTable("ShooterTable")
        # Create nt topics
        self._shooter_motor_velocity_topic = self._shooter_table.getDoubleTopic(
            "ShooterMotorVelocity"
        )

        # set nt defaults
        self._shooter_motor_velocity_pub = self._shooter_motor_velocity_topic.publish()
        self._shooter_motor_velocity_pub.set(self.target_velocity)

        # create nt subscribers
        self._shooter_motor_velocity_sub = self._shooter_motor_velocity_topic.subscribe(
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

        self.shooter_test_radius_topic = self._shooter_table.getDoubleTopic(
            "Shooter Test Radius"
        )
        self.shooter_test_radius_pub = self.shooter_test_radius_topic.publish()
        self.shooter_test_radius_sub = self.shooter_test_radius_topic.subscribe(2)

        # set up listeners

        self.lock = threading.Lock()

        def _on_shooter_rpm_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.target_velocity = event.data.value.getDouble()
                print(self.target_velocity)

        self.shooterListenerHandle = self._inst.addListener(
            self._shooter_motor_velocity_sub,
            ntcore.EventFlags.kValueAll,
            _on_shooter_rpm_changed,
        )

        def _on_shooter_p_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.p = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_p_changed_handle = self._inst.addListener(
            self._shooter_p_sub, ntcore.EventFlags.kValueAll, _on_shooter_p_changed
        )

        def _on_shooter_i_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.i = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_i_changed_handle = self._inst.addListener(
            self._shooter_i_sub, ntcore.EventFlags.kValueAll, _on_shooter_i_changed
        )

        def _on_shooter_d_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.d = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_d_changed_handle = self._inst.addListener(
            self._shooter_d_sub, ntcore.EventFlags.kValueAll, _on_shooter_d_changed
        )

        def _on_shooter_f_changed(event: ntcore.Event) -> None:
            with self.lock:
                self.shoot_config.f = event.data.value.getDouble()
                self.shoot_motor.apply_configs(self.shoot_config)
                self.f_shoot_motor.apply_configs(self.shoot_config)

        self.shooter_f_changed_handle = self._inst.addListener(
            self._shooter_f_sub, ntcore.EventFlags.kValueAll, _on_shooter_f_changed
        )

    def set_radius_pair(
        self, _r_dist: float, ignore_pairs: list[int]
    ) -> tuple[float, float] | None:

        if robot.TEST_MODE_ENABLED:
            self.closest_pair = (
                self.shooter_test_radius_sub.get(),
                self._shooter_motor_velocity_sub.get(),
            )
            return None

        min_r = 9999
        min_pair = (0, 0)

        for i in range(len(SHOOT_PAIRS)):
            if i in ignore_pairs:
                continue
            pair = SHOOT_PAIRS[i]
            delta = abs(_r_dist - pair[0])
            if delta < min_r:
                min_r = delta
                min_pair = pair

        if min_pair == (0, 0):
            return None

        self.closest_pair = min_pair

        return min_pair

    def set_shoot_voltage(self, volts: float) -> None:
        self.shoot_motor.set_voltage(volts)

    def set_shooter_velocity(self, velocity: float) -> None:
        print(velocity)
        self.target_velocity = velocity
        self.shoot_motor.set_velocity(velocity)

    def get_target_velocity_from_closest_pair(self) -> float:
        return self.closest_pair[1]

    def get_shoot_velocity_from_networktables(self) -> float:
        return self._shooter_motor_velocity_sub.get()

    def reset_shoot(self) -> None:
        self.shoot_encoder.set_position(0)

    def periodic(self) -> None:
        return

    @property
    def shoot_distance(self) -> float:
        return self.shoot_encoder.get_position()

    def get_shoot_velocity(self) -> float:
        return self.shoot_encoder.get_velocity()
