import commands2

from subsystems.shooter import ShooterSubsystem

SHOOT_VELOCITY = 4500


class SpinShooterCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem, rpm: float | None):
        super().__init__()
        self.shooter = shooter
        self.rpm = rpm
        self.addRequirements(shooter)

    def execute(self) -> None:
        if self.rpm is not None:
            self.shooter.shoot_motor.set_velocity(self.rpm)
            self.shooter.shoot_velocity = self.rpm
        else:
            self.shooter.set_shoot_velocity_from_networktables()

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.disable()
