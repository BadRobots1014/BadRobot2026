import commands2

import robot
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem

KICKER_VOLTAGE = 3


class ShootKickerCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(
        self, shooter: ShooterSubsystem, kicker: KickerSubsystem, invert: bool
    ):
        super().__init__()
        self.invert = invert
        self.shooter = shooter
        self.kicker = kicker
        self.addRequirements(shooter, kicker)

    def execute(self) -> None:
        if self.invert:
            self.kicker.set_kick_voltage(-KICKER_VOLTAGE)
        elif robot.TEST_MODE_ENABLED:
            self.kicker.set_kick_shoot_voltage_from_networktables()
        else:
            self.kicker.set_kick_voltage(KICKER_VOLTAGE)

    def end(self, interrupted: bool) -> None:
        self.kicker.kick_motor.disable()
