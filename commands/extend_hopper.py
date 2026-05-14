from commands2 import Command

from subsystems.hopper import HopperSubsystem


class ExtendHopperCommand(Command):
    def __init__(
        self,
        hopper: HopperSubsystem,
    ):
        """
        Use Network Tables to Extend / Retract hopper.

        :param extend: Whether to extend or retract hopper
        """
        super().__init__()
        self.hopper = hopper

        self.finished = False

        self.addRequirements(hopper)

    def execute(self) -> None:
        self.hopper.set_extension_voltage_from_networktable()

    def isFinished(self) -> bool:
        # Finish on limit
        if self.hopper.is_forward_extended():
            return True
        return False

    def end(self, interrupted: bool) -> None:
        self.hopper.has_hopper_extended = True
        self.hopper.set_extension_voltage(0)
