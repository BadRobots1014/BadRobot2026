import commands2
import wpimath.units

from subsystems.intake import IntakeSubsystem
from subsystems.talonFXIntake import TalonIntakeSubsystem

MOTOR_VOLTAGE = 4
INTAKE_VOLTAGE = 4

EXTEND_LENGTH_INCHES = 12


class ExtendHopperCommand(commands2.Command):
    def __init__(self, intake: IntakeSubsystem | TalonIntakeSubsystem, extend: bool):
        self.intake = intake
        self.extend = extend

    def execute(self) -> None:
        if self.extend:
            self.intake.set_extension_voltage_from_networktable()
        else:
            self.intake.set_retraction_voltage_from_networktable()

    def isFinished(self) -> bool:
        # if (self.extend and self.intake.forward_extended()) or (
        #     not self.extend and self.intake.backward_extended()
        # ):
        #     return True
        # return False
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
        self.intake.set_extension_voltage(0)
