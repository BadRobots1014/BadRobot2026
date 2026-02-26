from typing import Tuple

import ntcore
from ntcore import NetworkTableInstance
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d


class Limelight:
    def __init__(self, enabled: bool = True, name: str = "limelight") -> None:
        self.enabled = True
        self.name = name

        # setup network tables
        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable(name)

        # setup enabled listener
        self.enabled_topic = self.nt_table.getBooleanTopic("enabled")
        self.enabled_pub = self.enabled_topic.publish()
        self.enabled_pub.set(enabled)
        self.enabled_listener = self.nt_inst.addListener(
            self.enabled_topic, ntcore.EventFlags.kValueAll, self._enabled_changed
        )

        # Getters

        # returns [x, y, x, roll, pitch, yaw, latency]
        self.pose_sub = self.nt_table.getDoubleArrayTopic(
            "botpose_orb_wpiblue"
        ).subscribe([0] * 7)
        # MegaTag Standard Deviations [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
        self.stddevs_sub = self.nt_table.getDoubleArrayTopic("stddevs").subscribe(
            [0] * 12
        )
        # tv = target valid
        self.tv_sub = self.nt_table.getIntegerTopic("tv").subscribe(0)
        # tc = count
        # self.tc_sub = self.nt_table.getIntegerTopic("tc").subscribe(0)

        # Setters

        # Used in robot_orientation_set
        self.orientation_set_pub = self.nt_table.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()

    def _enabled_changed(self, event: ntcore.Event):
        self.enabled = event.data.value.getBoolean()

    def array_to_pose2d(self, arr: list[float]):
        return Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]))

    def vision_measurement_valid(self) -> bool:
        return self.tv_sub.get() == 1 and self.enabled

    # algorithm is used to tell the kalman filter how much to trust the pose estimation. lower is more confidant
    def get_deviation(self) -> Tuple[float, float, float]:
        arr = self.stddevs_sub.get()
        # Only use MT2x, MT2y
        # yaw standard deviation needs to be really high so the kalman filter ignores the yaw from limelight
        return arr[6], arr[7], 99999

    def get_vision_measurement(
        self,
    ) -> tuple[Pose2d, float, tuple[float, float, float]]:

        # get pose array
        arr = self.pose_sub.get()

        pose = self.array_to_pose2d(arr)
        # arr[6] is latency in ms, and is used to compute absolute timestamp of pose estimate
        timestamp = Timer.getFPGATimestamp() - (arr[6] / 1000.0)
        deviation = self.get_deviation()
        return pose, timestamp, deviation

    # Set Robot Orientation and angular velocities in degrees and degrees per second
    def robot_orientation_set(
        self,
        yaw: float,
    ):
        self.orientation_set_pub.set([yaw, 0, 0, 0, 0, 0])
