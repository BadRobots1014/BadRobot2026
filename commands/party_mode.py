import commands2

from subsystems.lights import Lights
from subsystems.music import Music


class PartyMode(commands2.Command):
    def __init__(self, light_system: Lights, music_system: Music):
        super().__init__()
        self.light_system = light_system
        self.music_system = music_system
        self.saturation = 255
        self.value = 255

    def initialize(self):
        print("PARTY MODE USED")
        self.light_system.set_rainbow(255, 255, 5)
        self.music_system.play_song()

    def end(self, interrupted):
        self.music_system.stop_song()

    def isFinished(self):
        return self.music_system.song_finished()
