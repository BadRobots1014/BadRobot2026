import commands2
from commands2.button import Trigger

import robot_class
from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from commands.shimmy import Shimmy
from commands.strafe import Strafe
from controllers.controller import ControllerDefault
from drive_wrapper import NUDGE_SPEED, MAX_SPEED
from routines.auto_shoot_with_intake import AutoShootWithIntake
from routines.dump_routine import DumpRoutine
from routines.goto_and_shoot import GotoAndShootRoutine
from routines.shoot_when_ready import ShootWhenReady
from subsystems.custom_controller import CustomController
from robot_class import RobotClass
from wpilib import DriverStation


class AuxController(ControllerDefault):
    def __init__(self, robot: RobotClass):
        super().__init__(self.AUXILIARY_PORT, robot)

    def configure(self):
        self.controller.bind_pov_up("Manual extend hopper").whileTrue(
            ExtendHopperCommand(self.robot.hopper)
        )

        # Spin up shooter L2
        self.controller.create_axis(
            self.L2_TRIGGER_AXIS, "shoot when ready", self.AXIS_THRESHOLD_VALUE
        ).whileTrue(
            ShootWhenReady(
                self.robot.shooter, self.robot.kicker, self.robot.conveyor, self.robot.intake, rpm=3300
            ),
        )

        # Run kicker wheel when ready R2
        self.controller.create_axis(
            self.R2_TRIGGER_AXIS,
            "goto and shoot when ready (dangerous)",
            self.AXIS_THRESHOLD_VALUE,
        ).whileTrue(
            GotoAndShootRoutine(
                self.robot.shooter,
                self.robot.kicker,
                self.robot.conveyor,
                self.robot.intake,
                self.robot.drive_wrapper,
                self.robot.get_hub,
                self.robot.is_blue,
            )
            # goto_radius
        )

        self.controller.create_button(
            self.L1_BUTTON, "shoot when ready (rpm=None)"
        ).whileTrue(
            ShootWhenReady(
                self.robot.shooter, self.robot.kicker, self.robot.conveyor, self.robot.intake, rpm=None
            )
        )

        # Intake wheel in (HOLD)
        intake_wheel_in = RunIntakeCommand(self.robot.intake, dump=False)
        self.controller.create_button(
            self.CROSS_BUTTON, "Intake wheel in"
        ).whileTrue(ExtendHopperCommand(self.robot.hopper).andThen(intake_wheel_in))

        # Intake wheel dump (HOLD)
        intake_wheel_out = DumpRoutine(self.robot.intake, self.robot.kicker, self.robot.conveyor)
        self.controller.create_button(
            self.CIRCLE_BUTTON, "Intake wheel dump"
        ).whileTrue(intake_wheel_out)

        # Intake wheel down up
        self.controller.create_button(
            self.SQUARE_BUTTON, "intake pulse"
        ).whileTrue(AutoShootWithIntake(self.robot.intake))