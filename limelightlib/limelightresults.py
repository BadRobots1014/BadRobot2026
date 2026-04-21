import time
from typing import Any


class GeneralResult:
    def __init__(self, results: dict[str, Any]) -> None:
        self.barcode: list[Any] = results.get("Barcode", [])

        self.classifierResults: list[ClassifierResult] = [
            ClassifierResult(x) for x in results.get("Classifier", [])
        ]
        self.detectorResults: list[DetectorResult] = [
            DetectorResult(x) for x in results.get("Detector", [])
        ]
        self.fiducialResults: list[FiducialResult] = [
            FiducialResult(x) for x in results.get("Fiducial", [])
        ]
        self.retroResults: list[RetroreflectiveResult] = [
            RetroreflectiveResult(x) for x in results.get("Retro", [])
        ]

        self.botpose: list[float] = results.get("botpose", [])
        self.botpose_wpiblue: list[float] = results.get("botpose_wpiblue", [])
        self.botpose_wpired: list[float] = results.get("botpose_wpired", [])
        self.stdev_mt1: list[float] = results.get("stdev_mt1", [])

        self.botpose_orb: list[float] = results.get("botpose_orb", [])
        self.botpose_orb_wpiblue: list[float] = results.get("botpose_orb_wpiblue", [])
        self.botpose_orb_wpired: list[float] = results.get("botpose_orb_wpired", [])
        self.stdev_mt2: list[float] = results.get("stdev_mt2", [])

        self.capture_latency: float = results.get("cl", 0)
        self.pipeline_id: int = results.get("pID", 0)

        self.robot_pose_target_space: list[float] = results.get("t6c_rs", [])
        self.targeting_latency: float = results.get("tl", 0)
        self.timestamp: float = results.get("ts", 0)
        self.validity: int = results.get("v", 0)

        self.parse_latency: float = 0.0


class RetroreflectiveResult:
    def __init__(self, d: dict[str, Any]) -> None:
        self.points: list[Any] = d["pts"]
        self.camera_pose_target_space: list[float] = d["t6c_ts"]
        self.robot_pose_field_space: list[float] = d["t6r_fs"]
        self.robot_pose_target_space: list[float] = d["t6r_ts"]
        self.target_pose_camera_space: list[float] = d["t6t_cs"]
        self.target_pose_robot_space: list[float] = d["t6t_rs"]

        self.target_area: float = d["ta"]
        self.target_x_degrees: float = d["tx"]
        self.target_x_pixels: float = d["txp"]
        self.target_y_degrees: float = d["ty"]
        self.target_y_pixels: float = d["typ"]


class FiducialResult:
    def __init__(self, d: dict[str, Any]) -> None:
        self.fiducial_id: int = d["fID"]
        self.family: str = d["fam"]
        self.points: list[Any] = d["pts"]
        self.skew: float = d["skew"]

        self.camera_pose_target_space: list[float] = d["t6c_ts"]
        self.robot_pose_field_space: list[float] = d["t6r_fs"]
        self.robot_pose_target_space: list[float] = d["t6r_ts"]
        self.target_pose_camera_space: list[float] = d["t6t_cs"]
        self.target_pose_robot_space: list[float] = d["t6t_rs"]

        self.target_area: float = d["ta"]
        self.target_x_degrees: float = d["tx"]
        self.target_x_pixels: float = d["txp"]
        self.target_y_degrees: float = d["ty"]
        self.target_y_pixels: float = d["typ"]


class DetectorResult:
    def __init__(self, d: dict[str, Any]) -> None:
        self.class_name: str = d["class"]
        self.class_id: int = d["classID"]
        self.confidence: float = d["conf"]

        self.points: list[Any] = d["pts"]

        self.target_area: float = d["ta"]
        self.target_x_degrees: float = d["tx"]
        self.target_x_pixels: float = d["txp"]
        self.target_y_degrees: float = d["ty"]
        self.target_y_pixels: float = d["typ"]


class ClassifierResult:
    def __init__(self, d: dict[str, Any]) -> None:
        self.class_name: str = d["class"]
        self.class_id: int = d["classID"]
        self.confidence: float = d["conf"]


def parse_results(json_data: dict[str, Any] | None) -> GeneralResult | None:
    start: float = time.time()

    if json_data is None:
        return None

    result = GeneralResult(json_data)
    result.parse_latency = (time.time() - start) * 1000
    return result
