from commands2.sysid import SysIdRoutine

from commands.extend_hopper import ExtendHopperCommand
from commands.run_conveyor import RunConveyor
from commands.run_intake import RunIntakeCommand
from commands.run_kicker import RunKickerCommand
from commands.run_shooter import RunShooterCommand
from controllers.controller import ControllerDefault
from subsystems import custom_controller


class TestController(ControllerDefault):
    def __init__(self, robot):
        super().__init__(self.TEST_PORT, robot)

    def configure(self):

        self.controller.create_axis(
            self.R2_TRIGGER_AXIS, "shoot", self.AXIS_THRESHOLD_VALUE
        ).whileTrue(RunShooterCommand(self.robot.shooter, rpm=None))

        self.controller.create_axis(
            self.L2_TRIGGER_AXIS, "extend hopper test", self.AXIS_THRESHOLD_VALUE
        ).whileTrue(ExtendHopperCommand(self.robot.hopper))

        self.controller.create_button(self. L1_BUTTON, "kicker").whileTrue(
            RunKickerCommand(self.robot.kicker, invert=False)
        )

        self.controller.create_button(self.R1_BUTTON, "kicker invert").whileTrue(
            RunKickerCommand(self.robot.kicker, invert=True)
        )

        self.controller.create_button(self.TRIANGLE_BUTTON, "conveyor").whileTrue(
            RunConveyor(self.robot.conveyor, shoot_direction=True)
        )

        self.controller.create_button(self.SQUARE_BUTTON, "conveyor invert").whileTrue(
            RunConveyor(self.robot.conveyor, shoot_direction=False)
        )

        self.controller.create_button(self.CROSS_BUTTON, "intake").whileTrue(
            RunIntakeCommand(self.robot.intake, dump=False)
        )

        self.controller.create_button(self.CIRCLE_BUTTON, "intake invert").whileTrue(
            RunIntakeCommand(self.robot.intake, dump=True)
        )

        self.robot.drive_wrapper.drivetrain.register_telemetry(self.robot.drive_wrapper.logger.telemeterize)
        custom_controller.write_binds()

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self.controller.button(self.SHARE_BUTTON)).whileTrue(
            self.robot.drive_wrapper.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self.controller.button(self.OPTIONS_BUTTON)).whileTrue(
            self.robot.drive_wrapper.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self.controller.button(self.L3_BUTTON)).whileTrue(
            self.robot.drive_wrapper.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self.controller.button(self.R3_BUTTON)).whileTrue(
            self.robot.drive_wrapper.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )