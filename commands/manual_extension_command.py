import commands2

from subsystems.intake import IntakeSubsystem


class ManualExtensionCommand(commands2.Command):
    def __init__(self, intake: IntakeSubsystem, extend: bool):
        super().__init__()
        self.extend = extend
        self.intake = intake

    def execute(self) -> None:
        if self.extend:
            self.intake.set_extention_voltage_from_networktable()
        else:
            self.intake.set_retraction_voltage_from_networktable()

    def end(self, interrupted: bool) -> None:
        self.intake.set_extension_voltage(0)
