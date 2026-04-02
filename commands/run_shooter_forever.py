from commands.run_shooter import RunShooterCommand
from subsystems.shooter import ShooterSubsystem


class RunShooterCommandForever(RunShooterCommand):
    def __init__(self, shooter: ShooterSubsystem, rpm: int):
        super().__init__(shooter, rpm)

    def isFinished(self) -> bool:
        return False
