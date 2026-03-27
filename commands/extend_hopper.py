import commands2
from wpilib import SmartDashboard
from wpimath import controller

import robot
from subsystems import pilights
from subsystems.hopper import HopperSubsystem

Kp, Ki, Kd, Kv = 0.2, 0, 0, 1


EXTEND_LENGTH_INCHES = 12


class ExtendHopperCommand(commands2.Command):
    def __init__(
        self,
        hopper: HopperSubsystem,
        lights: pilights.PiLights,
        extend: bool,
        max_distance_limit: float | None = None,
        extension_voltage: float | None = None,
    ):
        """
        Extends or Retracts the hopper until hardware limit / distance limit hit.

        :param extend: Whether to extend or retract hopper
        :param max_distance_limit: Forced stop distance, in motor revolutions.
        :param extension_voltage: TESTING ONLY; constant voltage to use during extension
        """
        super().__init__()
        self.hopper = hopper
        self.lights = lights

        self.extend = extend
        self.pid = controller.PIDController(Kp, Ki, Kd)
        self.voltage = extension_voltage
        self.max_distance_limit = max_distance_limit
        self.intiial_pos = self.hopper.get_extension_position()

        # Voltage must be positive
        if self.voltage is not None:
            assert self.voltage >= 0

        # PID tuning current in testing mode only
        if not robot.TEST_MODE_ENABLED:
            SmartDashboard.putData("Hopper PID", self.pid)

        self.addRequirements(hopper)

    def execute(self) -> None:
        # Test Mode Logic
        if not robot.TEST_MODE_ENABLED:
            # Use PID for setting voltage
            if self.voltage is None:
                output = self.pid.calculate(
                    self.hopper.get_extension_position(),
                    0 if not self.extend else self.hopper.get_max_extension_value(),
                )
                self.hopper.set_extension_voltage(
                    output + Kv * (-1 if not self.extend else 1)
                )
                self.lights.set_state(
                    pilights.LEDState.HOPPER_EXTEND
                    if self.extend
                    else pilights.LEDState.HOPPER_RETRACT
                )

                SmartDashboard.putNumber("PID output", output)
                SmartDashboard.putNumber(
                    "Goal",
                    0 if not self.extend else self.hopper.get_max_extension_value(),
                )
                SmartDashboard.putNumber(
                    "Position", self.hopper.get_extension_position()
                )
            # Use passed constant voltage
            else:
                self.hopper.set_extension_voltage(
                    self.voltage * (1 if self.extend else -1)
                )
                self.lights.set_state(
                    pilights.LEDState.HOPPER_EXTEND
                    if self.extend
                    else pilights.LEDState.HOPPER_RETRACT
                )
        # Normal Extend Logic, use Network Tables
        elif self.extend:
            self.hopper.set_extension_voltage_from_networktable()
        # Normal Retract Logic, use Network Tables
        else:
            self.hopper.set_retraction_voltage_from_networktable()

    def isFinished(self) -> bool:
        # Finish on distance travelled
        if self.max_distance_limit is not None:
            distance = self.hopper.get_extension_position() - self.intiial_pos
            distance_magnitude = abs(distance)
            if distance_magnitude > self.max_distance_limit:
                return True
        # Finish on limit
        if (self.extend and self.hopper.forward_extended()) or (
            not self.extend and self.hopper.backward_extended()
        ):
            self.lights.set_state(
                pilights.LEDState.HOPPER_EXTENDED
                if self.extend
                else pilights.LEDState.HOPPER_RETRACTED
            )
            return True
        return False

    def end(self, interrupted: bool) -> None:
        self.hopper.set_extension_voltage(0)
