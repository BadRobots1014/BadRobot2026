import commands2

from subsystems.shooter import ShooterSubsystem
from subsystems.kicker import KickerSubsystem


class KickerShootWhenReadyCommand(commands2.Command):
    # pass in parent subsystem
    def __init__(self, shooter: ShooterSubsystem, kicker: KickerSubsystem) -> None:
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.addRequirements(self.shooter, self.kicker)
        # make sure to add requirements to parent subsystem here

    # runs every scheduled tick (think of it as a while true)
    def execute(self) -> None:
        self.shooter.set_shoot_velocity_from_networktables()
        if self.shooter.shoot_encoder.get_velocity() > self.shooter.shoot_velocity:
            self.kicker.set_kick_shoot_voltage_from_networktables()
        else:
            pass

    # boolean condition to check if the command is finished (needed for running commands in series)
    def isFinished(self) -> bool:
        return False

    # code that runs after the command is finished
    def end(self, interrupted: bool) -> None:
        self.kicker.set_kick_voltage(0)
        self.shooter.set_shoot_voltage(0)
