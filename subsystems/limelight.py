import json

import commands2.subsystem
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d

from generated import limelight_pipelines
from generated.limelight_pipelines import APRILTAG_PIPELINE
from limelightlib.limelight import Limelight


class LimelightSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.ll_fuel = Limelight("10.10.14.16")
        self.ll_tag = Limelight("10.10.14.14")

        print("limelights initialized")

        self.apriltag_pipeline = limelight_pipelines.APRILTAG_PIPELINE

        self.ll_fuel.update_pipeline(
            json.dumps(limelight_pipelines.DETECTOR_PIPELINE), flush=1
        )  # persist
        self.ll_tag.update_pipeline(
            json.dumps(self.apriltag_pipeline), flush=1
        )  # persist
        # default tag pipeline will contain the modified tag list for auto
        print("limelight pipelines pushed")

    def set_teleop_id_filters(self) -> None:
        self.apriltag_pipeline["fiducial_idfilters"] = ""
        self.ll_tag.update_pipeline(json.dumps(self.apriltag_pipeline))

    def set_auto_id_filters(self) -> None:
        self.apriltag_pipeline["fiducial_idfilters"] = APRILTAG_PIPELINE[
            "fiducial_idfilters"
        ]
        self.ll_tag.update_pipeline(json.dumps(self.apriltag_pipeline))

    def disabled_throttle(self) -> None:
        self.ll_tag.update_throttle(59)
        self.ll_fuel.update_throttle(23)

    def teleop_throttle(self) -> None:
        self.ll_tag.update_throttle(0)
        self.ll_fuel.update_throttle(0)

    def set_imumode(self, imumode: int) -> None:
        self.ll_tag.update_imumode(imumode)

    # algorithm is used to tell the kalman filter how much to trust the pose estimation. lower is more confidant
    def get_deviation(self) -> tuple[float, float, float]:
        arr = self.ll_tag.stddevs_sub.get()
        # Only use MT2x, MT2y
        # yaw standard deviation needs to be really high so the kalman filter ignores the yaw from limelight
        return arr[6], arr[7], 99999

    def array_to_pose2d(self, arr: list[float]) -> Pose2d:
        return Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]))

    def get_vision_measurement(
        self,
    ) -> tuple[Pose2d, float, tuple[float, float, float]]:
        # get pose array
        arr = self.ll_tag.robot_pose_mt2_sub.get()

        pose = self.array_to_pose2d(arr)
        # arr[6] is latency in ms, and is used to compute absolute timestamp of pose estimate
        timestamp = Timer.getFPGATimestamp() - (arr[6] / 1000.0)
        deviation = self.get_deviation()
        return pose, timestamp, deviation

    def set_robot_orientation(self, yaw: float) -> None:
        self.ll_tag.robot_orientation_set(yaw)

    def periodic(self) -> None:
        pass
