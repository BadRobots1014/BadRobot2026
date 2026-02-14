import wpilib
from ntcore import NetworkTableInstance

from hardware.base.encoder import Encoder
from hardware.base.motor import Motor
from hardware.impl.spark_flex_motor import SparkFlexMotor
import math


class Shooter:
    # def __init__(self, shoot_motor_id: int, turn_motor_id: int):
    def __init__(
        self,
        shoot_motor: Motor,
        kick_motor: Motor,
        kick_encoder: Encoder,
        shoot_encoder: Encoder,
    ):
        self.shoot_motor = shoot_motor
        self.kick_motor = kick_motor

        self.shoot_encoder = shoot_encoder
        self.kick_encoder = kick_encoder

        self.shoot_velocity = 0
        self.kick_velocity = 0

        self.time_of_stall = -1
        self.start_unjam = -1

        self._inst = NetworkTableInstance.getDefault()
        self._shooter_table = self._inst.getTable("ShooterTable")
        # Create nt topics
        self._shooter_motor_velocity_topic = self._shooter_table.getDoubleTopic(
            "ShooterMotorVelocity"
        )
        self._kicker_motor_velocity_topic = self._shooter_table.getDoubleTopic(
            "KickerMotorVelocity"
        )

        # create nt subscribers
        self._shooter_motor_velocity_sub = self._shooter_motor_velocity_topic.subscribe(
            0.0
        )
        self._kicker_motor_velocity_sub = self._kicker_motor_velocity_topic.subscribe(
            0.0
        )

        # set nt defaults
        shooter_motor_velocity_pub = self._shooter_motor_velocity_topic.publish()
        shooter_motor_velocity_pub.set(0.0)
        kicker_motor_velocity_pub = self._kicker_motor_velocity_topic.publish()
        kicker_motor_velocity_pub.set(0.0)

    def set_shoot_voltage(self, volts: float):
        self.shoot_motor.set_voltage(volts)

    def set_shoot_velocity(self, velocity: float):
        self.shoot_velocity = velocity
        self.shoot_motor.set_velocity(velocity)

    def set_shoot_velocity_from_networktables(self):
        self.set_shoot_velocity(self._shooter_motor_velocity_sub.get())

    def set_kick_voltage(self, volts: float):
        self.kick_motor.set_voltage(volts)

    def set_kick_velocity(self, velocity: float):
        self.kick_velocity = velocity
        self.kick_motor.set_velocity(velocity)

    def set_kick_velocity_from_networktables(self):
        self.set_kick_velocity(self._kicker_motor_velocity_sub.get())

    def reset_shoot(self):
        self.shoot_encoder.set_position(0)

    def reset_kick(self):
        self.kick_encoder.set_position(0)

    def kick_unjam(self):
        if self.time_of_stall == -1 and self.kick_encoder.get_velocity() < 50:
            self.time_of_stall = wpilib.RobotController.getFPGATime()
            return
        time_stalled = wpilib.RobotController.getFPGATime() - self.time_of_stall
        if (
            self.time_of_stall != -1
            and self.kick_encoder.get_velocity() < 50
            and time_stalled > 1
        ):
            self.start_unjam = wpilib.RobotController.getFPGATime()
            self.kick_motor.set_velocity(-self.kick_velocity)
            return
        time_unjamming = wpilib.RobotController.getFPGATime() - self.start_unjam
        if time_unjamming > 1:
            self.kick_motor.set_velocity(self.kick_velocity)
            return
        return

    def periodic(self) -> None:
        self.kick_unjam()

    @property
    def shoot_distance(self) -> float:
        return self.shoot_encoder.get_position()

    @property
    def kick_distance(self) -> float:
        return self.kick_encoder.get_position()

    @property
    def shoot_voltage(self) -> float:
        return self.shoot_motor.get_voltage()

    @property
    def kick_voltage(self) -> float:
        return self.kick_motor.get_voltage()
