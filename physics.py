from pyfrc.physics import core

from robot import MyRobot


class PhysicsEngine(core.PhysicsEngine):
    def __init__(self, physics_controller, robot: "MyRobot"):
        self.physics_controller = physics_controller
        return

    def update_sim(self, now, tm_diff):
        """
        Update the simulation state based on the robot's current state.
        """
        return
