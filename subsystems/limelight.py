import json

import commands2.subsystem

from generated import limelight_pipelines
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
        self.ll_tag.update_pipeline(json.dumps(self.apriltag_pipeline))  # persist

    def disabled_throttle(self) -> None:
        self.ll_tag.update_throttle(59)
        self.ll_fuel.update_throttle(23)

    def teleop_throttle(self) -> None:
        self.ll_tag.update_throttle(0)
        self.ll_fuel.update_throttle(0)

    def set_imumode(self, imumode: int) -> None:
        self.ll_tag.update_imumode(imumode)

    def periodic(self) -> None:
        pass
