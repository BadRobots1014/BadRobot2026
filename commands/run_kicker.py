import commands2

import robot
from subsystems.kicker import KickerSubsystem

KICKER_VOLTAGE = 3


class RunKickerCommand(commands2.Command):
    kicker: KickerSubsystem

    def __init__(self, kicker: KickerSubsystem, invert: bool):
        """
        Run the kicker motor.
        """
        super().__init__()
        self.invert = invert
        self.kicker = kicker
        self.addRequirements(kicker)

    def execute(self) -> None:
        if robot.TEST_MODE_ENABLED:
            if self.invert:
                self.kicker.set_kick_dump_voltage_from_networktables()
            else:
                self.kicker.set_kick_shoot_voltage_from_networktables()
        elif self.invert:
            self.kicker.set_kick_voltage(-KICKER_VOLTAGE)
        else:
            self.kicker.set_kick_voltage(KICKER_VOLTAGE)

    def end(self, interrupted: bool) -> None:
        self.kicker.kick_motor.disable()
