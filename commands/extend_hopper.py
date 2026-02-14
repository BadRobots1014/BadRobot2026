import commands2

from subsystems.intake import Intake


MOTOR_VOLTAGE = 4

class ExtendHopper(commands2.Command):
    def __init__(self, intake: Intake):
        super().__init__()
        self.intake = intake
        self.extend = False if self.intake.forward_extended() else True

        if self.extend:
            self.intake.set_extension_voltage(-MOTOR_VOLTAGE)
        else:
            self.intake.set_intake_voltage(MOTOR_VOLTAGE)

    def isFinished(self) -> bool:
        if (self.extend and self.intake.forward_extended()) or (
            self.extend and self.intake.backward_extended()
        ):
            self.intake.set_extension_voltage(0)
            return True

        return False
