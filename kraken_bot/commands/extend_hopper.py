import commands2

from kraken_bot.subsystems.intake import Intake
from commands2 import Command

class ExtendHopper(commands2.Command):
    def __init__(self, intake: Intake):
        super().__init__()
        Intake.set_extension_voltage(-4)

    def isFinished(self) -> bool:
        if Intake.is_extended:
            Intake.set_extension_voltage(0)

        return Intake.is_extended()
