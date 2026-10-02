from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand

from commands.run_conveyor import RunConveyor
from commands.run_intake import RunIntakeCommand
from commands.run_kicker import RunKickerCommand
from commands.run_shooter import RunShooterCommand
from subsystems.conveyor import ConveyorSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem


class ShootWhenReady(SequentialCommandGroup):
    def __init__(
        self,
        shooter: ShooterSubsystem,
        kicker: KickerSubsystem,
        conveyor: ConveyorSubsystem,
        intake: IntakeSubsystem,
        rpm: int | None,
    ) -> None:
        super().__init__(
            RunShooterCommand(shooter, desired_rpm=rpm, end_after_reach=True),
            ParallelCommandGroup(
                RunShooterCommand(shooter, desired_rpm=rpm),
                RunKickerCommand(kicker, invert=False),
                RunIntakeCommand(intake, dump=False),
                WaitCommand(0.2).andThen(RunConveyor(conveyor, shoot_direction=True)),
            ),
        )
