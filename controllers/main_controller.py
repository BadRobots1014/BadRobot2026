import commands2
from commands2.button import Trigger

import robot_class
from commands.shimmy import Shimmy
from commands.strafe import Strafe
from controllers.controller import ControllerDefault
from drive_wrapper import NUDGE_SPEED, MAX_SPEED
from subsystems.custom_controller import CustomController
from robot_class import RobotClass
from wpilib import DriverStation

class MainController(ControllerDefault):
    def __init__(self, robot: RobotClass):
        super().__init__(self.DRIVER_PORT, robot)

    def configure(self):
        strafe_l = Strafe(
            self.robot.drive_wrapper,
            self.robot.shooter,
            self.robot.get_hub,
            clockwise=True
        )
        strafe_r = Strafe(
            self.robot.drive_wrapper,
            self.robot.shooter,
            self.robot.get_hub,
            clockwise=False
        )

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.robot.drive_wrapper.drivetrain.setDefaultCommand(
            self.robot.drive_wrapper.field_centric_drive(
                self.getLeftX() * self.robot.drive_wrapper.max_speed,
                self.getLeftY() * self.robot.drive_wrapper.max_speed,
                self.getRightX() * self.robot.drive_wrapper.max_angular_speed
            )
        )

        self.controller.create_button(
            self.L1_BUTTON, "Strafe Left Around Tower"
        ).whileTrue(strafe_l)
        self.controller.create_button(
            self.R1_BUTTON, "Strafe Right Around Tower"
        ).whileTrue(strafe_r)

        # NUDGING

        # POV up - drive forward
        self.controller.create_axis(
            self.R2_TRIGGER_AXIS, "nudge backwards", self.AXIS_THRESHOLD_VALUE
        ).whileTrue(
            self.robot.drive_wrapper.robot_centric_drive(self.robot.drive_wrapper.nudge_speed, 0)
        )

        # POV down - drive backward
        self.controller.create_axis(
            self.L2_TRIGGER_AXIS, "nudge backwards", self.AXIS_THRESHOLD_VALUE
        ).whileTrue(
            self.robot.drive_wrapper.robot_centric_drive(-self.robot.drive_wrapper.nudge_speed, 0)
        )

        # POV right - drive right
        self.controller.bind_pov_right("nudge right").whileTrue(
            self.robot.drive_wrapper.robot_centric_drive(0, -self.robot.drive_wrapper.nudge_speed)
        )

        # POV left - drive left
        self.controller.bind_pov_left("nudge left").whileTrue(
            self.robot.drive_wrapper.robot_centric_drive(0, self.robot.drive_wrapper.nudge_speed)
        )

        # POINTING

        self.controller.create_button(
            self.TRIANGLE_BUTTON, "point forward"
        ).whileTrue(
            self.robot.drive_wrapper.theta_centric_drive(0, self.getLeftY() * self.robot.drive_wrapper.max_speed, self.getLeftX() * self.robot.drive_wrapper.max_speed)
        )

        self.controller.create_button(
            self.CROSS_BUTTON, "point backward"
        ).whileTrue(
            self.robot.drive_wrapper.theta_centric_drive(180, self.getLeftY() * self.robot.drive_wrapper.max_speed, self.getLeftX() * self.robot.drive_wrapper.max_speed)
        )

        self.controller.create_button(
            self.SQUARE_BUTTON, "point left"
        ).whileTrue(
            self.robot.drive_wrapper.theta_centric_drive(90, self.getLeftY() * self.robot.drive_wrapper.max_speed, self.getLeftX() * self.robot.drive_wrapper.max_speed)
        )

        self.controller.create_button(
            self.CIRCLE_BUTTON, "point right"
        ).whileTrue(
            self.robot.drive_wrapper.theta_centric_drive(270, self.getLeftY() * self.robot.drive_wrapper.max_speed, self.getLeftX() * self.robot.drive_wrapper.max_speed)
        )

        # TODO: MAKE THIS A COMMAND INSTEAD OF THIS JARGON

        # Reset the field-centric heading on Options button press
        self.controller.create_button(self.OPTIONS_BUTTON, "Reset Heading").onTrue(
            self.robot.drive_wrapper.drivetrain.runOnce(self.robot.drive_wrapper.drivetrain.seed_field_centric).andThen(
                commands2.InstantCommand(self.robot.camera_ll4.set_imu_mode(1))
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        Trigger(DriverStation.isDisabled).whileTrue(
            self.robot.drive_wrapper.drivetrain.apply_request(lambda: self.robot.drive_wrapper.drivetrain.idle()).ignoringDisable(
                doesRunWhenDisabled=True
            )
        )

        self.controller.bind_pov_down("waggle").whileTrue(
            Shimmy(self.robot.drive_wrapper.drivetrain)
        )