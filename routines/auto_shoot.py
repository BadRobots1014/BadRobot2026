from commands2 import ParallelCommandGroup

from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from subsystems.estimate_rpm import EstimateRPM
from subsystems.intake import IntakeSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.pilights import PiLights
from subsystems.shooter import ShooterSubsystem


class ShootInPlaceRoutine(ParallelCommandGroup):
    def __init__(
        self,
        talon_intake: IntakeSubsystem,
        shooter: ShooterSubsystem,
        kicker: KickerSubsystem,
        lights: PiLights,
        estimate_rpm: EstimateRPM,
    ) -> None:
        super().__init__()
        self.addCommands(
            KickerShootWhenReadyCommand(
                shooter, kicker, lights, estimate_rpm.calculate_rpm()
            ),
        )
