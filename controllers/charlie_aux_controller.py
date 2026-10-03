import commands2

from commands.extend_hopper import ExtendHopperCommand
from commands.goto_shoot_radius import GotoShootRadius
from controllers.controller import ControllerDefault
from robot_class import RobotClass
from routines.auto_shoot_with_intake import AutoShootWithIntake
from routines.dump_routine import DumpRoutine
from routines.shoot_when_ready import ShootWhenReady


class CharlieAuxController(ControllerDefault):
    def __init__(self, robot: RobotClass):
        super().__init__(self.AUXILIARY_PORT, robot)

    def configure(self) -> None:
        intake_wheel_out = DumpRoutine(
            self.robot.intake, self.robot.kicker, self.robot.conveyor
        )
        self.controller.create_axis(
            self.R2_TRIGGER_AXIS, "Intake wheel dump", self.AXIS_THRESHOLD_VALUE
        ).whileTrue(intake_wheel_out)

        self.controller.create_axis(
            self.L2_TRIGGER_AXIS,
            "shoot when ready (rpm=None)",
            self.AXIS_THRESHOLD_VALUE,
        ).whileTrue(
            ShootWhenReady(
                self.robot.shooter,
                self.robot.kicker,
                self.robot.conveyor,
                self.robot.intake,
                rpm=3200,
            )
        )

        self.controller.create_button(
            self.L1_BUTTON, "Shoot when ready (auto RPM)"
        ).whileTrue(
            ShootWhenReady(
                self.robot.shooter,
                self.robot.kicker,
                self.robot.conveyor,
                self.robot.intake,
                rpm=None,
            )
        )

        self.controller.create_button(self.CROSS_BUTTON, "GoTo").whileTrue(
            GotoShootRadius(
                self.robot.drive_wrapper,
                self.robot.shooter,
                self.robot.get_hub,
                self.robot.is_blue,
            )
        )

        self.controller.bind_pov_up("Manual extend hopper").whileTrue(
            ExtendHopperCommand(self.robot.hopper)
        )

        self.controller.bind_pov_down("Unjam").whileTrue(
            AutoShootWithIntake(self.robot.intake)
        )

        self.controller.create_button(self.OPTIONS_BUTTON, "Reset Heading").onTrue(
            self.robot.drive_wrapper.drivetrain.runOnce(
                self.robot.drive_wrapper.drivetrain.seed_field_centric
            ).andThen(commands2.InstantCommand(self.robot.camera_ll4.set_imu_mode(1)))
        )
