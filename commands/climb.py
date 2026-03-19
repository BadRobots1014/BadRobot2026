import commands2

from subsystems.climber import ClimberSubsystem


class ClimbCommand(commands2.Command):
    climber_subsystem: ClimberSubsystem

    def __init__(self, climber_subsystem: ClimberSubsystem, extend: bool = True):
        super().__init__()
        self.climber_subsystem = climber_subsystem
        self.extend = extend
        self.addRequirements(climber_subsystem)

    def execute(self) -> None:
        if self.extend:
            self.climber_subsystem.motor_extend()
        else:
            self.climber_subsystem.motor_retract()

    def end(self, interrupted: bool) -> None:
        self.climber_subsystem.motor_stop()
