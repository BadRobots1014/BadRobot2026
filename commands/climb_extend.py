import commands2

from subsystems.climber import ClimberSubsystem


class ClimbCommand(commands2.Command):
    climber_subsystem: ClimberSubsystem

    def __init__(self, climber_subsystem: ClimberSubsystem):
        super().__init__()
        self.climber_subystem = ClimberSubsystem

    def execute(self) -> None:
        self.climber_subsystem.motor_extend()

    def end(self, interrupted: bool) -> None:
        self.climber_subsystem.motor_stop()
