from collections.abc import Callable

from commands2 import (
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup, WaitCommand,
)
from wpimath.geometry import Translation2d

from commands.goto_shoot_radius import GotoShootRadius
from commands.run_conveyor import RunConveyor
from commands.run_kicker import RunKickerCommand
from commands.run_shooter import RunShooterCommand
from drive_wrapper import DriveWrapper
from routines.shoot_when_ready import ShootWhenReady
from subsystems.conveyor import ConveyorSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem

class GotoAndShootRoutine(SequentialCommandGroup):
    def __init__(
        self,
        shooter: ShooterSubsystem,
        kicker: KickerSubsystem,
        conveyor: ConveyorSubsystem,
        intake: IntakeSubsystem,
        drivetrain: DriveWrapper,
        hub: Callable[[], Translation2d],
        blue_alliance: bool
    ):
        super().__init__(
            ParallelCommandGroup(
                GotoShootRadius(drivetrain, shooter, hub, blue_alliance),
                RunShooterCommand(shooter, desired_rpm=None, persist=True),
            ),
            ParallelCommandGroup(
                RunKickerCommand(kicker, invert=False),
                WaitCommand(0.2).andThen(RunConveyor(conveyor, shoot_direction=True)),
                GotoShootRadius(drivetrain, shooter, hub, blue_alliance),
            ),
        )