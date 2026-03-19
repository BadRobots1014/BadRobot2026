from commands2 import (
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
)
from wpimath._controls._controls.controller import PIDController

from commands.goto_shoot_radius import GotoShootRadius
from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from commands.spin_shooter import SpinShooterCommand
from kraken_container import BLUE_HUB_TRANSLATION
from subsystems import pilights
from subsystems.shooter import ShooterSubsystem
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain


class GotoAndShoot(SequentialCommandGroup):
    def __init__(
        self,
        _shooter: ShooterSubsystem,
        drivetrain: CommandSwerveDrivetrain,
        lights: pilights.PiLights,
        drive_pid: PIDController,
        rotate_pid: PIDController,
    ):
        super().__init__(
            ParallelDeadlineGroup(
                GotoShootRadius(
                    drivetrain,
                    _shooter,
                    BLUE_HUB_TRANSLATION,
                    drive_pid,
                    rotate_pid,
                ),
                SpinShooterCommand(_shooter, rpm=None),
            ),
            ParallelCommandGroup(
                GotoShootRadius(
                    drivetrain,
                    _shooter,
                    BLUE_HUB_TRANSLATION,
                    drive_pid,
                    rotate_pid,
                ),
                KickerShootWhenReadyCommand(_shooter, lights, rpm=None),
            ),
        )
