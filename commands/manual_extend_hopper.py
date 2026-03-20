from commands2 import Command
from wpilib import SmartDashboard
from wpimath import controller

from commands.extend_hopper import Kd, Ki, Kp
from subsystems import pilights
from subsystems.hopper import HopperSubsystem


class ManualExtendHopperCommand(Command):
    def __init__(
        self,
        hopper: HopperSubsystem,
        lights: pilights.PiLights,
        extend: bool,
        positive_voltage: float | None = None,
        positive_distance_limit: float | None = None,
    ):
        # Cannot pass in a negative voltage
        super().__init__()
        self.hopper = hopper
        self.lights = lights

        self.extend = extend
        self.pid = controller.PIDController(Kp, Ki, Kd)
        self.voltage = positive_voltage
        self.distance_limit = positive_distance_limit
        self.intiial_pos = self.hopper.get_extension_position()

        if self.voltage is not None:
            assert self.voltage >= 0

        self.addRequirements(hopper)

        SmartDashboard.putData("Hopper PID", self.pid)

    def execute(self) -> None:
        if self.extend:
            self.hopper.set_extension_voltage_from_networktable()
        else:
            self.hopper.set_retraction_voltage_from_networktable()

    def isFinished(self) -> bool:
        # Finish on distance travelled
        if self.distance_limit is not None:
            distance = self.hopper.get_extension_position() - self.intiial_pos
            if distance * (-1 if not self.extend else 1) > self.distance_limit:
                return True
        # Finish on limit
        if (self.extend and self.hopper.forward_extended()) or (
            not self.extend and self.hopper.backward_extended()
        ):
            self.lights.set_state(
                pilights.LEDState.HOPPER_EXTENDED
                if self.extend
                else pilights.LEDState.HOPPER_RETRACTED
            )
            return True
        return False

    def end(self, interrupted: bool) -> None:
        self.hopper.set_extension_voltage(0)
