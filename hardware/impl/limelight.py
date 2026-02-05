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

        # returns [x, y, x, roll, pitch, yaw, latency]
        self.pose_sub = self.nt_table.getDoubleArrayTopic("botpose_wpiblue").subscribe(
            [0] * 7
        )
        # MegaTag Standard Deviations [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
        self.stddevs_sub = self.nt_table.getDoubleArrayTopic("stddevs").subscribe(
            [0] * 12
        )
        # tv = target valid
        self.tv_sub = self.nt_table.getBooleanTopic("tv").subscribe(False)
        # tc = tag count
        self.tc_sub = self.nt_table.getIntegerTopic("tc").subscribe(0)

    def _enabled_changed(self, event: ntcore.Event):
        self.enabled = event.data.value.getBoolean()

    def array_to_pose2d(self, arr: list[float]):
        return Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]))

    def vision_measurement_valid(self) -> bool:
        return self.tv_sub.get() and self.enabled

    # algorithm is used to tell the kalman filter how much to trust the pose estimation. lower is more confidant
    def get_deviation(self) -> Tuple[float, float, float]:
        arr = self.stddevs_sub.get()
        # Only use MT2x, MT2y, MT2yaw standard deviations
        return arr[6], arr[7], arr[11]

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
