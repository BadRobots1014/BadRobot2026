from commands2 import Command

from subsystems.hopper import HopperSubsystem


class ExtendHopperCommand(Command):
    def __init__(
        self,
        hopper: HopperSubsystem,
        extend: bool,
    ):
        """
        Use Network Tables to Extend / Retract hopper.

        :param extend: Whether to extend or retract hopper
        """
        super().__init__()
        self.hopper = hopper

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
            return True
        return False

    def end(self, interrupted: bool) -> None:
        self.hopper.set_extension_voltage(0)
