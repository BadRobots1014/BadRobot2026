import logging
from pathlib import Path

from commands2 import Subsystem
from phoenix6.orchestra import Orchestra
import wpilib

from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class MusicSubsystem(Subsystem):
    def __init__(self, drivetrain: CommandSwerveDrivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.orchestra = Orchestra()

        for module in self.drivetrain.modules:
            self.orchestra.add_instrument(module.drive_motor)
            self.orchestra.add_instrument(module.steer_motor)

        deploy_path = Path(wpilib.getDeployDirectory())
        file_name = "still_alive.chrp"
        full_path = deploy_path / file_name

        status = self.orchestra.load_music(str(full_path))

        if not status.is_ok():
            logging.error(f"Music failed to load: {status.name}")  # noqa: LOG015
        else:
            logging.info("Music loaded successfully!")  # noqa: LOG015

    def play_song(self) -> None:
        self.orchestra.play()

    def stop_song(self) -> None:
        self.orchestra.stop()

    def song_finished(self) -> bool:
        return not self.orchestra.is_playing()
