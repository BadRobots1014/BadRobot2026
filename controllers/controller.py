import math
from typing import Literal

from wpimath.geometry import Rotation2d

from robot_class import RobotClass
from subsystems.custom_controller import CustomController

class ControllerDefault:

    # joysticks
    DRIVER_PORT = 0
    AUXILIARY_PORT = 1
    TEST_PORT = 2
    FLIGHT_STICK_PORT = 3
    TURN_TO_THETA_PORT = 4

    # Controller axis mappings
    LEFT_X_AXIS = 0
    LEFT_Y_AXIS = 1
    RIGHT_X_AXIS = 4
    RIGHT_Y_AXIS = 5
    L2_TRIGGER_AXIS = 2
    R2_TRIGGER_AXIS = 3

    FLIGHT_STICK_POV_VECTORS = {
        0: (1, 0),
        45: (1, 0),
        90: (0, -1),
        135: (-1, 0),
        180: (-1, 0),
        225: (-1, 0),
        270: (0, 1),
        315: (1, 0),
    }

    FLIGHT_STICK_X_AXIS = 0
    FLIGHT_STICK_Y_AXIS = 1
    FLIGHT_STICK_YAW_AXIS = 2

    AXIS_THRESHOLD_VALUE = 0.67

    # Controller button mappings
    CROSS_BUTTON = 1
    CIRCLE_BUTTON = 2
    SQUARE_BUTTON = 3
    TRIANGLE_BUTTON = 4
    L1_BUTTON = 5
    R1_BUTTON = 6
    SHARE_BUTTON = 7
    OPTIONS_BUTTON = 8
    L3_BUTTON = 9
    R3_BUTTON = 10

    POV_UP = 0
    POV_RIGHT = 90
    POV_LEFT = 270
    POV_DOWN = 180

    SLOW_SPEED_JOYSTICK_MODIFIER = 0.5

    def __init__(self, port, robot: RobotClass):
        self.controller = CustomController(port)
        self.robot = robot
        self.last_angle = Rotation2d.fromRotations(0)

    def getLeftX(self) -> float:
        raw = -(self.controller.getRawAxis(self.LEFT_X_AXIS) ** 3)
        if self.robot.slow_mode:
            raw *= self.SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getLeftY(self) -> float:
        raw = -(self.controller.getRawAxis(self.LEFT_Y_AXIS) ** 3)
        if self.robot.slow_mode:
            raw *= self.SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getRightX(self) -> float:
        raw = -(self.controller.getRawAxis(self.RIGHT_X_AXIS) ** 3)
        if self.robot.slow_mode:
            raw *= self.robot.drive_wrapper.SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getRightY(self) -> float:
        raw = -(self.controller.getRawAxis(self.RIGHT_Y_AXIS) ** 3)
        if self.robot.slow_mode:
            raw *= self.SLOW_SPEED_JOYSTICK_MODIFIER
        return raw

    def getTargetAngle(self) -> Rotation2d:
        x = self.getRightX()
        y = self.getRightY()
        if math.sqrt(x * x + y * y) > self.robot.drive_wrapper.TURN_TO_THETA_DEADBAND:
            self.last_angle = Rotation2d.fromRotations(math.atan2(x, y) / (2 * math.pi))
        return self.last_angle