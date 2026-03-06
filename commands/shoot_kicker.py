import commands2

from subsystems.shooter import ShooterSubsystem

KICKER_VOLTAGE = 3


class ShootKickerCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem):
        super().__init__()
        self.shooter = shooter

    def execute(self) -> None:
        self.shooter.set_kick_voltage_from_networktables()

    def end(self, interrupted: bool) -> None:
        self.shooter.kick_motor.disable()
