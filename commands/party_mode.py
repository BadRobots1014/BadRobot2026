import commands2

from subsystems.lights import Lights


class PartyMode(commands2.InstantCommand):
    def __init__(self, light_system: Lights):
        super().__init__()
        self.light_system = light_system
        self.saturation = 255
        self.value = 255

    def initialize(self):
        self.light_system.set_rainbow(255, 255)
