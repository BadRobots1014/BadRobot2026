import commands2

import robot
from subsystems.shooter import ShooterSubsystem
DEFAULT_RPM = 3200

class RunShooterCommand(commands2.Command):
    shooter: ShooterSubsystem

    def __init__(
        self,
        shooter: ShooterSubsystem,
        desired_rpm: int | None,
        end_after_reach: bool = False,
    ):
        super().__init__()
        self.shooter = shooter
        self.rpm = 0 if desired_rpm is None else desired_rpm
        self.desired_rpm = desired_rpm
        self.end_after_reach = end_after_reach
        self.addRequirements(shooter)

    def execute(self) -> None:
        if not robot.TEST_MODE_ENABLED:
            if self.desired_rpm is not None:
                self.shooter.set_shoot_velocity(self.rpm)
            else:
                self.rpm = self.shooter.get_shoot_velocity_from_closest_pair()
                if self.rpm == 0:
                    self.rpm = DEFAULT_RPM
                self.shooter.set_shoot_velocity(self.rpm)
        else:
            self.rpm = self.shooter.get_shoot_velocity_from_networktables()
            self.shooter.set_shoot_velocity(self.rpm)

    # we're up to speed
    def isFinished(self) -> bool:
        print(
            self.end_after_reach,
            self.shooter.shoot_encoder.get_velocity(),
            self.shooter.shoot_velocity,
            self.shooter.get_shoot_velocity_from_closest_pair(),
        )
        return (
            self.end_after_reach
            and self.shooter.shoot_encoder.get_velocity() >= float(self.rpm - 50)
        )

    def end(self, interrupted: bool) -> None:
        self.shooter.shoot_motor.stop_motor()
