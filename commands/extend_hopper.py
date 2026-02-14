import commands2

from kraken_bot.subsystems.intake import Intake
from commands2 import Command


class ExtendHopper(commands2.Command):
    def __init__(self, intake: Intake):
        super().__init__()
        self.intake = intake
        self.extend = False if self.intake.forward_extended() else True

        if self.extend:
            self.intake.set_extension_voltage(-4)
        else:
            self.intake.set_intake_voltage(0)

    def isFinished(self) -> bool:
        if (self.extend and self.intake.forward_extended()) or (
            self.extend and self.intake.backward_extended()
        ):
            self.intake.set_extension_voltage(0)
            return True

        return False
