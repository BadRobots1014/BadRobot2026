from commands2 import Command, Subsystem, StartEndCommand
from phoenix6.orchestra import Orchestra
import wpilib
import os
import logging


class Music(Subsystem):
    def __init__(self, motors, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.orchestra = Orchestra()

        for motor in motors:
            self.orchestra.add_instrument(motor)

        deploy_path = wpilib.getDeployDirectory()
        file_name = "still_alive.chrp"
        full_path = os.path.join(deploy_path, file_name)

        status = self.orchestra.load_music(full_path)

        if not status.is_ok():
            logging.error(f"Music failed to load: {status.name}")
        else:
            logging.info("Music loaded successfully!")

    def play_song(self) -> Command:
        return StartEndCommand(
            self.orchestra.play, self.orchestra.stop, self, self.drivetrain
        ).until(self.song_finished)

    def song_finished(self):
        return not self.orchestra.is_playing()
