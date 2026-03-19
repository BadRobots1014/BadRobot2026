import commands2

from subsystems.music import MusicSubsystem
from subsystems.pilights import LEDState, PiLights


class PartyModeCommand(commands2.Command):
    def __init__(self, light_system: PiLights, music_system: MusicSubsystem):
        super().__init__()
        self.light_system = light_system
        self.music_system = music_system
        self.saturation = 255
        self.value = 255

        self.addRequirements(
            self.light_system, self.music_system, self.music_system.drivetrain
        )

    def initialize(self) -> None:
        print("PARTY MODE USED")
        self.light_system.set_state(LEDState.PARTY_MODE)
        self.music_system.play_song()

    def end(self, interrupted: bool) -> None:
        self.music_system.stop_song()

    def isFinished(self) -> bool:
        return self.music_system.song_finished()
