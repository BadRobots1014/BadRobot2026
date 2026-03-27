from commands2 import (
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
)
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d

from commands.goto_shoot_radius import GotoShootRadius
from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from commands.run_shooter import RunShooterCommand
from subsystems import pilights
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class GotoAndShootRoutine(SequentialCommandGroup):
    def __init__(
        self,
        _shooter: ShooterSubsystem,
        _kicker: KickerSubsystem,
        drivetrain: CommandSwerveDrivetrain,
        lights: pilights.PiLights,
        drive_pid: PIDController,
        rotate_pid: PIDController,
        hub: Translation2d,
    ):
        super().__init__(
            ParallelDeadlineGroup(
                GotoShootRadius(
                    drivetrain,
                    _shooter,
                    hub,
                    drive_pid,
                    rotate_pid,
                ),
                RunShooterCommand(_shooter, rpm=None),
            ),
            ParallelCommandGroup(
                GotoShootRadius(
                    drivetrain,
                    _shooter,
                    hub,
                    drive_pid,
                    rotate_pid,
                ),
                KickerShootWhenReadyCommand(_shooter, _kicker, lights, rpm=None),
            ),
        )
