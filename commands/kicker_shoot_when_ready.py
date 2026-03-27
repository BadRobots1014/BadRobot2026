import commands2

from subsystems import pilights
from subsystems.kicker import KickerSubsystem
from subsystems.shooter import ShooterSubsystem


class KickerShootWhenReadyCommand(commands2.Command):
    # pass in parent subsystem
    def __init__(
        self,
        shooter: ShooterSubsystem,
        kicker: KickerSubsystem,
        lights: pilights.PiLights,
        rpm: int | None,
    ) -> None:
        """
        Spin up shooter to desired `rpm`. Once reached, run kicker motor.

        :param rpm: target rpm of shooter. Defaults to Network Table value.
        """
        super().__init__()

        self.shooter = shooter
        self.kicker = kicker
        self.lights = lights
        self.rpm = rpm

        self.addRequirements(self.shooter)

    def execute(self) -> None:
        self.lights.set_state(pilights.LEDState.SHOOTER_REV)
        # Use specified RPM
        if self.rpm is not None:
            self.shooter.shoot_motor.set_velocity(self.rpm)
            self.shooter.shoot_velocity = self.rpm
        # Use Network Table RPM
        else:
            self.shooter.set_shoot_velocity_from_networktables()

        # Spin Kicker when above desired rpm
        if self.shooter.shoot_encoder.get_velocity() > self.shooter.shoot_velocity:
            self.kicker.set_kick_shoot_voltage_from_networktables()
            self.lights.set_state(pilights.LEDState.SHOOTER_READY)
        else:
            pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.kicker.kick_motor.set_voltage(0)
        self.shooter.shoot_motor.set_voltage(0)
