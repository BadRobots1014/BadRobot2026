from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.run_conveyor import RunConveyor
from commands.run_kicker import RunKickerCommand
from commands.run_shooter import RunShooterCommand
from subsystems.conveyor import ConveyorSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem


class ShootWhenReady(SequentialCommandGroup):
    def __init__(
        self,
        shooter: ShooterSubsystem,
        kicker: KickerSubsystem,
        conveyor: ConveyorSubsystem,
        rpm: int | None,
    ) -> None:
        super().__init__(
            RunShooterCommand(shooter, rpm=rpm),
            ParallelCommandGroup(
                RunShooterCommand(shooter, rpm=rpm),
                RunKickerCommand(kicker, invert=False),
                RunConveyor(conveyor, shoot_direction=True),
            ),
        )
