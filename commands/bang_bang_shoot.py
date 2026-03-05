import commands2

from subsystems.shooter import ShooterSubsystem


class BangBangShootCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem):
        super().__init__()
        self.shooter = shooter

    def execute(self) -> None:
        desired_velocity = self.shooter.get_shoot_velocity_from_networktables()
        desired_velocity = 4500
        print(self.shooter.f_shoot_motor.get_encoder().get_velocity())
        if desired_velocity > self.shooter.f_shoot_motor.get_encoder().get_velocity():
            self.shooter.set_shoot_voltage(12)
        else:
            self.shooter.set_shoot_voltage(0)

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.disable()
