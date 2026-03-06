import commands2
import wpimath.units

from subsystems.intake import IntakeSubsystem

MOTOR_VOLTAGE = 4
INTAKE_VOLTAGE = 4

EXTEND_LENGTH_INCHES = 12


class ExtendHopperCommand(commands2.Command):
    def __init__(self, intake: IntakeSubsystem, extend: bool):
        super().__init__()
        self.intake = intake
        self.extend = extend

        if self.extend:
            self.intake.set_extension_voltage(-MOTOR_VOLTAGE)
        else:
            self.intake.set_extension_voltage(MOTOR_VOLTAGE)

    def isFinished(self) -> bool:
        if (self.extend and self.intake.forward_extended()) or (
            not self.extend and self.intake.backward_extended()
        ):
            self.intake.set_extension_voltage(0)
            return True

        return False

    def end(self, interrupted: bool) -> None:
        pose = self.intake.pos_subscriber.get()
        pose[0] += (
            wpimath.units.inchesToMeters(EXTEND_LENGTH_INCHES)
            if self.extend
            else -wpimath.units.inchesToMeters(EXTEND_LENGTH_INCHES)
        )
        # recast so compiler knows it's a list
        self.intake.pose_publisher.set([float(x) for x in pose])
