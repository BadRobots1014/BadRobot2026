from hardware.base.motor import Motor

class Intake:
    def __init__(
        self,
        extended: False,
        intake: Motor,
        kicker: Motor,
        extension: Motor,
    ):
        self.extended = extended

        self.intake = intake
        self.kicker = kicker
        self.extension = extension

    def set_intake_voltage(self, voltage: float):
        self.intake.set_voltage(voltage)

    def set_kicker_voltage(self, voltage: float):
        self.kicker.set_voltage(voltage)

    def set_extension_voltage(self, voltage: float):
        self.extension.set_voltage(voltage)

    def extend(self, extended: bool):
        self.extended = extended

    @property
    def intake_voltage(self):
        return self.intake.get_voltage()

    @property
    def kicker_voltage(self):
        return self.kicker.get_voltage()