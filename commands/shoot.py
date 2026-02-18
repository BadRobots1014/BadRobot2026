import commands2

from subsystems.shooter import Shooter


class Shoot(commands2.Command):
    shooter: Shooter

    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = shooter

    def execute(self):
        self.shooter.set_shoot_velocity_from_networktables()

    def end(self, interrupted: bool):
        self.shooter.shoot_motor.disable()
