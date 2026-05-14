import math

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
                self.shooter.set_shooter_velocity(self.rpm)
            else:
                self.shooter.set_shooter_velocity(
                    self.shooter.get_target_velocity_from_closest_pair()
                )
        else:
            self.shooter.set_shooter_velocity(
                self.shooter.get_shoot_velocity_from_networktables()
            )

    # we're up to speed
    def isFinished(self) -> bool:
        # print(math.fabs(self.shooter.shoot_encoder.get_velocity()), self.shooter.target_velocity)
        return (
            math.fabs(self.shooter.shoot_encoder.get_velocity())
            >= self.shooter.target_velocity + 50
        )

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.set_voltage(0)
