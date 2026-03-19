import commands2

from subsystems.shooter import ShooterSubsystem


class BangBangShootCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem, velocity: float | None):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(shooter)
        self.desired_velocity = velocity

    def execute(self) -> None:
        if self.desired_velocity is None:
            velocity = self.shooter.shoot_velocity
        else:
            velocity = self.desired_velocity

        if velocity > self.shooter.get_shoot_velocity():
            self.shooter.set_shoot_voltage(12)
        else:
            self.shooter.set_shoot_voltage(0)

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.disable()
