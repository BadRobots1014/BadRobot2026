from commands2 import ParallelCommandGroup

from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from commands.shoot import ShootCommand
from subsystems.shooter import ShooterSubsystem


class ShootInPlace(ParallelCommandGroup):
    def __init__(self, shoot: ShooterSubsystem):
        self.addRequirements(shoot)
        super().__init__()
        self.addCommands(KickerShootWhenReadyCommand(shoot), ShootCommand(shoot))
