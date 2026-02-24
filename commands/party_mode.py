import commands2

from subsystems.lights import LightSubsystem


class PartyModeCommand(commands2.InstantCommand):
    def __init__(self, light_system: LightSubsystem):
        super().__init__()
        self.light_system = light_system
        self.saturation = 255
        self.value = 255

    def initialize(self):
        print("PARTY MODE USED")
        self.light_system.set_rainbow(255, 255, 5)
