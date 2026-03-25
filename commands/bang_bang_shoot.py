import commands2

from subsystems.shooter import ShooterSubsystem


class BangBangShootCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem, velocity: float | None):
        """
        Apply Bang algorithm to maintain shooter rpm (velocity)

        :param velocity: desired rpm to maintain. Defaults to shooter.shoot_velocity
        """
        super().__init__()
        self.shooter = shooter
        self.addRequirements(shooter)
        if velocity is None:
            self.desired_velocity = self.shooter.shoot_velocity
        else:
            self.desired_velocity = velocity

    def execute(self) -> None:
        if self.desired_velocity > self.shooter.get_shoot_velocity():
            self.shooter.set_shoot_voltage(12)
        else:
            self.shooter.set_shoot_voltage(0)

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.disable()
