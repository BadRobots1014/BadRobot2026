import commands2

from subsystems import pilights
from subsystems.shooter import ShooterSubsystem


class KickerShootWhenReadyCommand(commands2.Command):
    # pass in parent subsystem
    def __init__(self, shooter: ShooterSubsystem, lights: pilights.PiLights):
        super().__init__()
        self.shooter = shooter
        self.lights = lights
        self.addRequirements(self.shooter, self.lights)
        # make sure to add requirements to parent subsystem here

    # runs every scheduled tick (think of it as a while true)
    def execute(self) -> None:
        self.shooter.set_shoot_velocity_from_networktables()
        self.lights.set_state(pilights.LEDState.SHOOTER_REV)
        if self.shooter.shoot_encoder.get_velocity() > self.shooter.shoot_velocity:
            self.shooter.set_kick_shoot_voltage_from_networktables()
            self.lights.set_state(pilights.LEDState.SHOOTER_READY)
        else:
            pass

    # boolean condition to check if the command is finished (needed for running commands in series)
    def isFinished(self) -> bool:
        return False

    # code that runs after the command is finished
    def end(self, interrupted: bool) -> None:
        self.shooter.kick_motor.set_voltage(0)
        self.shooter.shoot_motor.set_voltage(0)
