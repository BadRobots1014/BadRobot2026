import wpilib
from commands2 import Subsystem
from ntcore import NetworkTableInstance

from hardware.base.encoder import Encoder
from hardware.base.motor import Motor
import rev
from rev import ResetMode, PersistMode

UNJAM_SPIN_TIME = 1  # time to spin to unjam in seconds
JAM_TIME = 1  # time to be considered jammed in seconds
JAM_RPM = 50  # rpm threshold to be considered jammed


class Shooter(Subsystem):
    def __init__(
        self,
        main_shoot_motor: Motor,
        follower_shoot_motor: Motor,
        shoot_encoder: Encoder,
        kick_motor: Motor,
        kick_encoder: Encoder,
    ):
        self.shoot_motor = main_shoot_motor
        self.f_shoot_motor = follower_shoot_motor

        self.kick_motor = kick_motor

        self.shoot_encoder = shoot_encoder
        self.kick_encoder = kick_encoder

        self.shoot_velocity = 4500
        self.kick_velocity = 600

        # tracks time for automatic jamming procedures
        self.time_of_stall = -1
        self.start_unjam = -1

        self.follower_config = rev.SparkFlexConfig()
        self.follower_config.inverted(True)
        self.follower_config.follow(self.shoot_motor.get_motor_controller())

        self.f_shoot_motor.get_motor_controller().configure(
            self.follower_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

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
        velocity = self._shooter_motor_velocity_sub.get()
        self.set_shoot_velocity(velocity)

    def set_kick_voltage(self, volts: float):
        self.kick_motor.set_voltage(volts)

    def set_kick_velocity(self, velocity: float):
        self.kick_velocity = velocity
        self.kick_motor.set_velocity(velocity)

    def set_kick_velocity_from_networktables(self):
        velocity = self._kicker_motor_velocity_sub.get()
        self.set_kick_velocity(velocity)

    def reset_shoot(self):
        self.shoot_encoder.set_position(0)

    def reset_kick(self):
        self.kick_encoder.set_position(0)

    def kick_unjam(self):
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
            self.kick_motor.set_velocity(-self.kick_velocity)
            return
        time_unjamming = wpilib.RobotController.getFPGATime() - self.start_unjam
        # go normal if unjamming for more than one second
        if time_unjamming > UNJAM_SPIN_TIME:
            self.kick_motor.set_velocity(self.kick_velocity)
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

    @property
    def shoot_voltage(self) -> float:
        return self.shoot_motor.get_voltage()

    @property
    def kick_voltage(self) -> float:
        return self.kick_motor.get_voltage()

    def get_shoot_velocity_from_networktables(self) -> float:
        velocity = self._shooter_motor_velocity_sub.get()
        return velocity
