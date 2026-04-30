import ntcore
from ntcore import NetworkTableInstance
from wpilib import DriverStation, Timer
from wpimath.geometry import Pose2d, Rotation2d


class Limelight:
    def __init__(
        self,
        name: str = "limelight",
        enabled: bool = True,
    ) -> None:
        self.enabled = True
        self.name = name

        self.captures = 0

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

        self.rewind_enable_topic = self.nt_table.getIntegerTopic("rewind_enable_set")
        self.rewind_enable_pub = self.rewind_enable_topic.publish()

        self.capture_rewind_topic = self.nt_table.getIntegerArrayTopic("capture_rewind")
        self.capture_rewind_pub = self.capture_rewind_topic.publish()

        if DriverStation.isFMSAttached():
            self.rewind_enable_pub.set(1)

        # Getters

        # returns [x, y, x, roll, pitch, yaw, latency]
        self.pose_topic = self.nt_table.getDoubleArrayTopic("botpose_wpiblue")
        self.pose_sub = self.pose_topic.subscribe([0] * 7)
        # MegaTag Standard Deviations [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
        self.stddevs_sub = self.nt_table.getDoubleArrayTopic("stddevs").subscribe(
            [0] * 12
        )
        # tv = target valid
        self.tv_sub = self.nt_table.getIntegerTopic("tv").subscribe(0)

        self.target_pose_sub = self.nt_table.getDoubleArrayTopic(
            "targetpose_robotspace"
        ).subscribe([0] * 7)

        # Setters

        # Used in robot_orientation_set
        self.orientation_set_pub = self.nt_table.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()

        # Used in set_throttle — controls frame skipping for LL4 thermal management
        self.throttle_set_pub = self.nt_table.getIntegerTopic("throttle_set").publish()

    def check_fms_enable_replay(self) -> None:
        if DriverStation.isFMSAttached() or True:
            self.rewind_enable_pub.set(1)

    def check_fms_capture_replay(self) -> None:
        if DriverStation.isFMSAttached() or True:
            self.capture_rewind_pub.set([self.captures, 165])
            self.captures += 1

    def _enabled_changed(self, event: ntcore.Event) -> None:
        self.enabled = event.data.value.getBoolean()

    def array_to_pose2d(self, arr: list[float]) -> Pose2d:
        return Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]))

    def vision_measurement_valid(self) -> bool:
        return self.tv_sub.get() == 1 and self.enabled

    # algorithm is used to tell the kalman filter how much to trust the pose estimation. lower is more confidant
    def get_deviation(self) -> tuple[float, float, float]:
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
    ) -> None:
        self.orientation_set_pub.set([yaw, 0, 0, 0, 0, 0])

    def set_imu_mode(self, imumode: int) -> None:
        self.nt_table.putNumber("imumode_set", imumode)

    def set_throttle(self, n: int) -> None:
        """Set LL4 frame-skip throttle for thermal management.

        Processes one frame after every n skipped frames.
        Recommended: 50-200 while disabled, 0 during active play.
        """
        self.throttle_set_pub.set(n)

    def set_auto_fiducial_id_filters(self) -> None:
        ids = [
            1,
            2,
            3,
            4,
            5,
            7,
            8,
            9,
            10,
            11,
            12,
            13,
            14,
            18,
            19,
            20,
            21,
            23,
            24,
            25,
            26,
            27,
            28,
            29,
            30,
        ]
        # ignore inner trench and climb tags
        self.nt_table.putNumberArray("fiducial_id_filters_set", ids)

    def set_teleop_fiducial_id_filters(self) -> None:
        ids = list(range(1, 33))
        # ignore inner trench and climb tags
        self.nt_table.putNumberArray("fiducial_id_filters_set", ids)
