from commands2 import Subsystem
from hardware.base.motorcontroller import MotorController
from hardware.base.switch import LimitSwitch

# Dumping velocity should be 1500


class IntakeSubsystem(Subsystem):
    def __init__(
        self,
        intake: MotorController,
        left: MotorController,
        right: MotorController,
        # forward: LimitSwitch,
        # backward: LimitSwitch,
    ) -> None:
        super().__init__()
        self.intake_motor = intake

        self.left = left
        self.right = right
        self.right.set_leader(self.left.get_motor_id(), True)
        #
        # self.forward = forward
        # self.backward = backward

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
        return False
        # return self.forward.get_state()

    def backward_extended(self) -> bool:
        return False
        # return self.backward.get_state()
