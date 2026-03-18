import commands2
from wpilib import SmartDashboard
from wpimath import controller, units

import robot
from subsystems.talonFXIntake import TalonIntakeSubsystem

MOTOR_VOLTAGE = 4
Kp, Ki, Kd, Kv = 0.2, 0, 0, 1


EXTEND_LENGTH_INCHES = 12


class ExtendHopperCommand(commands2.Command):
    def __init__(self, intake: TalonIntakeSubsystem, extend: bool):
        self.intake = intake
        self.extend = extend
        self.pid = controller.PIDController(Kp, Ki, Kd)

        SmartDashboard.putData("Hopper PID", self.pid)

    def execute(self) -> None:
        if not robot.TEST_MODE_ENABLED:
            # self.intake.set_extension_voltage(
            #     MOTOR_VOLTAGE * (-1 if not self.extend else 1)
            # )
            output = self.pid.calculate(
                self.intake.get_extension_position(),
                0 if not self.extend else self.intake.get_max_extension_value(),
            )
            self.intake.set_extension_voltage(
                output + Kv * (-1 if not self.extend else 1)
            )

            SmartDashboard.putNumber("PID output", output)
            SmartDashboard.putNumber(
                "Goal", 0 if not self.extend else self.intake.get_max_extension_value()
            )
            SmartDashboard.putNumber("Position", self.intake.get_extension_position())

        elif self.extend:
            self.intake.set_extension_voltage_from_networktable()
        else:
            self.intake.set_retraction_voltage_from_networktable()

    def isFinished(self) -> bool:
        if (self.extend and self.intake.forward_extended()) or (
            not self.extend and self.intake.backward_extended()
        ):
            return True
        return False

    def end(self, interrupted: bool) -> None:
        pose = self.intake.pos_subscriber.get()
        pose[0] += (
            units.inchesToMeters(EXTEND_LENGTH_INCHES)
            if self.extend
            else -units.inchesToMeters(EXTEND_LENGTH_INCHES)
        )
        # recast so compiler knows it's a list
        self.intake.pose_publisher.set([float(x) for x in pose])
        self.intake.set_extension_voltage(0)
