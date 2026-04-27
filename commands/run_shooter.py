import commands2

import robot
from subsystems.shooter import ShooterSubsystem


class RunShooterCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(self, shooter: ShooterSubsystem, rpm: int | None):
        super().__init__()
        self.shooter = shooter
        self.rpm = rpm
        self.addRequirements(shooter)

    def execute(self) -> None:
        if not robot.TEST_MODE_ENABLED:
            if self.rpm is not None:
                self.shooter.set_shoot_velocity(self.rpm)
            else:
                self.shooter.set_shoot_velocity(
                    self.shooter.get_shoot_velocity_from_closest_pair()
                )
        else:
            self.shooter.set_shoot_velocity(
                self.shooter.get_shoot_velocity_from_networktables()
            )
        self.shooter.shoot_motor.set_velocity(self.shooter.shoot_velocity)

    # we're up to speed
    def isFinished(self) -> bool:
        return self.shooter.shoot_encoder.get_velocity() > self.shooter.shoot_velocity

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.disable()
