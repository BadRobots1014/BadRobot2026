import commands2

from subsystems.shooter import Shooter


class Shoot(commands2.Command):
    shooter: Shooter

    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = shooter

    def execute(self):
        desired_velocity = self.shooter.get_shoot_velocity_from_networktables()
        if desired_velocity > self.shooter.shoot_velocity:
            self.shooter.set_shoot_voltage(12)
        else:
            self.shooter.set_shoot_voltage(0)

    def end(self, interrupted: bool):
        self.shooter.shoot_motor.disable()
