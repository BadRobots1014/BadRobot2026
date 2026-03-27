from commands2 import Command

from subsystems import pilights
from subsystems.hopper import HopperSubsystem


class ManualExtendHopperCommand(Command):
    def __init__(
        self,
        hopper: HopperSubsystem,
        lights: pilights.PiLights,
        extend: bool,
        max_distance_limit: float | None = None,
    ):
        """
        Use Network Tables to Extend / Retract hopper.

        :param extend: Whether to extend or retract hopper
        :param max_distance_limit: Forced stop distance, in motor revolutions.
        """
        super().__init__()
        self.hopper = hopper
        self.lights = lights

        self.extend = extend
        self.max_distance_limit = max_distance_limit
        self.intiial_pos = self.hopper.get_extension_position()

        self.addRequirements(hopper)

    def execute(self) -> None:
        if self.extend:
            self.hopper.set_extension_voltage_from_networktable()
        else:
            self.hopper.set_retraction_voltage_from_networktable()

    def isFinished(self) -> bool:
        # Finish on distance travelled
        if self.max_distance_limit is not None:
            distance = self.hopper.get_extension_position() - self.intiial_pos
            distance_magnitude = abs(distance)
            if distance_magnitude > self.max_distance_limit:
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
