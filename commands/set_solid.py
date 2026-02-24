import commands2

from subsystems.lights import LightSubsystem


class SetSolidCommand(commands2.InstantCommand):
    def __init__(self, light_system: LightSubsystem, r: int, g: int, b: int):
        super().__init__()
        self.light_system = light_system
        self.r = r
        self.g = g
        self.b = b

    def initialize(self):
        self.light_system.set_solid(self.r, self.g, self.b)
