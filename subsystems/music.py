from commands2 import Command, Subsystem, StartEndCommand
from phoenix6.orchestra import Orchestra
import wpilib
import os
import logging


class MusicSubsystem(Subsystem):
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.orchestra = Orchestra()

        for module in self.drivetrain.modules:
            self.orchestra.add_instrument(module.drive_motor)
            self.orchestra.add_instrument(module.steer_motor)

        deploy_path = wpilib.getDeployDirectory()
        file_name = "still_alive.chrp"
        full_path = os.path.join(deploy_path, file_name)

        status = self.orchestra.load_music(full_path)

        if not status.is_ok():
            logging.error(f"Music failed to load: {status.name}")

    def play_song(self):
        self.orchestra.play()

    def stop_song(self):
        self.orchestra.stop()

    def song_finished(self):
        return not self.orchestra.is_playing()
