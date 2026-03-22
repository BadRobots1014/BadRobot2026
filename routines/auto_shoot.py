from commands2 import ParallelCommandGroup

from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from routines.time_jiggle import TimeJiggle
from subsystems.kicker import KickerSubsystem
from subsystems.pilights import PiLights
from subsystems.shooter import ShooterSubsystem
from subsystems.talonFXIntake import TalonIntakeSubsystem
from subsystems.estimate_rpm import EstimateRPM


class ShootInPlace(ParallelCommandGroup):
    def __init__(
        self,
        talon_intake: TalonIntakeSubsystem,
        shooter: ShooterSubsystem,
        kicker: KickerSubsystem,
        lights: PiLights,
        estimate_rpm: EstimateRPM,
    ) -> None:
        super().__init__()
        self.addCommands(
            KickerShootWhenReadyCommand(shooter, kicker, lights, estimate_rpm.calculate_rpm()),
            TimeJiggle(talon_intake, lights),
        )