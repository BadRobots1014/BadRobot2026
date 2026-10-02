from commands.extend_hopper import ExtendHopperCommand
from commands.goto_shoot_radius import GotoShootRadius
from controllers.controller import ControllerDefault
from robot_class import RobotClass
from routines.auto_shoot_with_intake import AutoShootWithIntake
from routines.dump_routine import DumpRoutine
from routines.shoot_when_ready import ShootWhenReady


class AuxController(ControllerDefault):
    def __init__(self, robot: RobotClass):
        super().__init__(self.AUXILIARY_PORT, robot)

    def configure(self) -> None:
        intake_wheel_out = DumpRoutine(
            self.robot.intake, self.robot.kicker, self.robot.conveyor
        )
        self.controller.create_button(
            self.CIRCLE_BUTTON, "Intake wheel dump"
        ).whileTrue(intake_wheel_out)

        self.controller.create_button(
            self.L1_BUTTON, "shoot when ready (rpm=None)"
        ).whileTrue(
            ShootWhenReady(
                self.robot.shooter, self.robot.kicker, self.robot.conveyor, self.robot.intake, rpm=3300)
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
