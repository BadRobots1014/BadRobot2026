import logging
from pathlib import Path

from commands2 import Command, StartEndCommand, Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.orchestra import Orchestra
import wpilib

from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class MusicSubsystem(Subsystem):
    def __init__(self, motors: list[TalonFX], drivetrain: CommandSwerveDrivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.orchestra = Orchestra()

        for motor in motors:
            self.orchestra.add_instrument(motor)

        deploy_path = Path(wpilib.getDeployDirectory())
        file_name = "still_alive.chrp"
        full_path = deploy_path / file_name

        status = self.orchestra.load_music(str(full_path))

        if not status.is_ok():
            logging.error(f"Music failed to load: {status.name}")  # noqa: LOG015
        else:
            logging.info("Music loaded successfully!")  # noqa: LOG015

    def play_song(self) -> Command:
        return StartEndCommand(
            self.orchestra.play, self.orchestra.stop, self, self.drivetrain
        ).until(self.song_finished)

    def song_finished(self) -> bool:
        return not self.orchestra.is_playing()
