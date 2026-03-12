import commands2

import robot
from subsystems.shooter import ShooterSubsystem

SHOOT_VELOCITY = 4500


class ShootCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem):
        super().__init__()
        self.shooter = shooter

    def execute(self) -> None:
        if robot.TEST_MODE_ENABLED:
            self.shooter.set_shoot_velocity_from_networktables()
        else:
            self.shooter.set_shoot_velocity(SHOOT_VELOCITY)

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.disable()
