import commands2
from commands2.button import Trigger
from wpilib import DriverStation

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from controllers.controller import ControllerDefault
from robot_class import RobotClass
from routines.goto_and_shoot_shimmy import GotoAndShootShimmyRoutine
from routines.shoot_when_ready import ShootWhenReady


class CharlieMainController(ControllerDefault):
    def __init__(self, robot: RobotClass):
        super().__init__(self.DRIVER_PORT, robot)

    def configure(self) -> None:

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.robot.drive_wrapper.drivetrain.setDefaultCommand(
            self.robot.drive_wrapper.field_centric_drive_command(
                lambda: self.getLeftY() * self.robot.drive_wrapper.max_speed,
                lambda: self.getLeftX() * self.robot.drive_wrapper.max_speed,
                lambda: self.getRightX() * self.robot.drive_wrapper.max_angular_speed,
            )
        )

        # NUDGING

        # POV up - drive forward
        self.controller.bind_pov_up(
             "nudge forward"
        ).whileTrue(
            self.robot.drive_wrapper.robot_centric_drive_command(
                lambda: self.robot.drive_wrapper.nudge_speed, lambda: 0
            )
        )

        # POV down - drive backward
        self.controller.bind_pov_down(
            "nudge backwards"
        ).whileTrue(
            self.robot.drive_wrapper.robot_centric_drive_command(
                lambda: -self.robot.drive_wrapper.nudge_speed, lambda: 0
            )
        )

        # POV right - drive right
        self.controller.bind_pov_right("nudge right").whileTrue(
            self.robot.drive_wrapper.robot_centric_drive_command(
                lambda: 0, lambda: -self.robot.drive_wrapper.nudge_speed
            )
        )

        # POV left - drive left
        self.controller.bind_pov_left("nudge left").whileTrue(
            self.robot.drive_wrapper.robot_centric_drive_command(
                lambda: 0, lambda: self.robot.drive_wrapper.nudge_speed
            )
        )

        # Intake
        intake_wheel_in = RunIntakeCommand(self.robot.intake, dump=False)
        self.controller.create_axis(
            self.L2_TRIGGER_AXIS, "Intake wheel in", self.AXIS_THRESHOLD_VALUE
        ).whileTrue(ExtendHopperCommand(self.robot.hopper).andThen(intake_wheel_in))

        self.controller.create_axis(self.R2_TRIGGER_AXIS, "auto shoot", self.AXIS_THRESHOLD_VALUE).whileTrue(
            GotoAndShootShimmyRoutine(self.robot.shooter, self.robot.kicker, self.robot.conveyor, self.robot.intake, self.robot.drive_wrapper, self.robot.get_hub, self.robot.is_blue)
        )


        # # POINTING

        self.controller.create_button(self.TRIANGLE_BUTTON, "point forward").whileTrue(
            self.robot.drive_wrapper.theta_centric_drive_command(
                lambda: 0,
                lambda: self.getLeftY() * self.robot.drive_wrapper.max_speed,
                lambda: self.getLeftX() * self.robot.drive_wrapper.max_speed,
            )
        )

        self.controller.create_button(self.CROSS_BUTTON, "point backward").whileTrue(
            self.robot.drive_wrapper.theta_centric_drive_command(
                lambda: 180,
                lambda: self.getLeftY() * self.robot.drive_wrapper.max_speed,
                lambda: self.getLeftX() * self.robot.drive_wrapper.max_speed,
            )
        )

        self.controller.create_button(self.SQUARE_BUTTON, "point left").whileTrue(
            self.robot.drive_wrapper.theta_centric_drive_command(
                lambda: 90,
                lambda: self.getLeftY() * self.robot.drive_wrapper.max_speed,
                lambda: self.getLeftX() * self.robot.drive_wrapper.max_speed,
            )
        )

        self.controller.create_button(self.CIRCLE_BUTTON, "point right").whileTrue(
            self.robot.drive_wrapper.theta_centric_drive_command(
                lambda: 270,
                lambda: self.getLeftY() * self.robot.drive_wrapper.max_speed,
                lambda: self.getLeftX() * self.robot.drive_wrapper.max_speed,
            )
        )

        # # TODO: MAKE THIS A COMMAND INSTEAD OF THIS JARGON

        # Reset the field-centric heading on Options button press
        self.controller.create_button(self.OPTIONS_BUTTON, "Reset Heading").onTrue(
            self.robot.drive_wrapper.drivetrain.runOnce(
                self.robot.drive_wrapper.drivetrain.seed_field_centric
            ).andThen(commands2.InstantCommand(self.robot.camera_ll4.set_imu_mode(1)))
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        Trigger(DriverStation.isDisabled).whileTrue(
            self.robot.drive_wrapper.drivetrain.apply_request(
                lambda: self.robot.drive_wrapper.idle_request
            ).ignoringDisable(doesRunWhenDisabled=True)
        )
