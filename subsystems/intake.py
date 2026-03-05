from commands2 import Subsystem
from hardware.base.motorcontroller import MotorController
from hardware.base.switch import LimitSwitch
from ntcore import NetworkTableInstance

# Dumping velocity should be 1500


class IntakeSubsystem(Subsystem):
    def __init__(
        self,
        intake: MotorController,
        left: MotorController,
        right: MotorController,
        forward: LimitSwitch,
        backward: LimitSwitch,
        camera_name: str = "limelight",
    ) -> None:
        super().__init__()
        self.intake_motor = intake

        self.left = left
        self.right = right

        self.forward = forward
        self.backward = backward

        # setup network tables
        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable(camera_name)
        self.pose_publisher = self.nt_table.getDoubleArrayTopic(
            "camerapose_robotspace"
        ).publish()
        self.pos_subscriber = self.nt_table.getDoubleArrayTopic(
            "camerapose_robotspace"
        ).subscribe([0, 0, 0, 0, 0, 0])

    def set_intake_voltage(self, voltage: float):
        self.intake_motor.set_voltage(voltage)

    def set_intake_velocity(self, rpm: float):
        self.intake_motor.set_velocity(rpm)

    def set_extension_voltage(self, voltage: float):
        self.left.set_voltage(voltage)

    @property
    def intake_voltage(self):
        return self.intake_motor.get_voltage()

    @property
    def extension_voltage(self):
        return self.left.get_voltage()

    def forward_extended(self) -> bool:
        return self.forward.get_state()

    def backward_extended(self) -> bool:
        return self.backward.get_state()
