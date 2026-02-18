import commands2

from subsystems.shooter import Shooter


class Shoot_Kicker(commands2.Command):
    shooter: Shooter

    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = shooter

    def execute(self):
        self.shooter.set_kick_velocity_from_networktables()

    def end(self, interrupted: bool):
        self.shooter.kick_motor.disable()
