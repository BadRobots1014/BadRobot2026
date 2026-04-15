import commands2.subsystem
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
import math

class ExampleSubsystem(commands2.Subsystem):
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.running = False
        self.current_path = None

    # runs every scheduled tick
    def periodic(self) -> None:
        if not self.running:
            fuel_positions = []
            list_waypoints = []

            robot_pose = self.drivetrain.get_state().pose

            for pose in fuel_positions:
                rotated_pose = pose.rotateBy(robot_pose.rotation())
                list_waypoints.append(Pose2d(*(robot_pose.position() + rotated_pose.position()), robot_pose.rotation()))

            waypoints = PathPlannerPath.waypointsFromPoses(list_waypoints)
            constraints = PathConstraints(1.5, 1, 1.5, 1, 12)
            self.current_path = PathPlannerPath(waypoints, constraints, None, GoalEndState(0.0, robot_pose.rotation()))