from commands2 import Command

from subsystems import pilights
from subsystems.hopper import HopperSubsystem


class ManualExtendHopperCommand(Command):
    def __init__(
        self,
        hopper: HopperSubsystem,
        lights: pilights.PiLights,
        extend: bool,
    ):
        """
        Use Network Tables to Extend / Retract hopper.

        :param extend: Whether to extend or retract hopper
        """
        super().__init__()
        self.hopper = hopper
        self.lights = lights

        self.extend = extend
        self.intiial_pos = self.hopper.get_extension_position()

        self.addRequirements(hopper)

    def execute(self) -> None:
        if self.extend:
            self.hopper.set_extension_voltage_from_networktable()
        else:
            self.hopper.set_retraction_voltage_from_networktable()

    def isFinished(self) -> bool:
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
