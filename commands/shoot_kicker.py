import commands2

from subsystems.shooter import ShooterSubsystem

KICKER_VOLTAGE = 3

class ShootKickerCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem):
        super().__init__()
        self.shooter = shooter

    def execute(self):
        self.shooter.set_kick_voltage(KICKER_VOLTAGE)

    def end(self, interrupted: bool):
        self.shooter.kick_motor.disable()
