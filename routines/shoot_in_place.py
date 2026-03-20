from commands2 import ParallelCommandGroup

from commands.kicker_shoot_when_ready import KickerShootWhenReadyCommand
from routines.time_jiggle import TimeJiggleRoutine
from subsystems.hopper import HopperSubsystem
from subsystems.kicker import KickerSubsystem
from subsystems.pilights import PiLights
from subsystems.shooter import ShooterSubsystem


class ShootInPlaceRoutine(ParallelCommandGroup):
    def __init__(
        self,
        hopper: HopperSubsystem,
        shooter: ShooterSubsystem,
        kicker: KickerSubsystem,
        lights: PiLights,
    ) -> None:
        super().__init__()
        self.addCommands(
            KickerShootWhenReadyCommand(shooter, kicker, lights, 3300),
            TimeJiggleRoutine(hopper, lights),
        )
