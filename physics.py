#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

from __future__ import annotations

import typing

from pyfrc.physics.core import PhysicsInterface

from kraken_container import KrakenRobotContainer

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, _physics_controller: PhysicsInterface, robot: MyRobot):
        # Must be using KrakenBot
        assert isinstance(robot.container, KrakenRobotContainer), (
            "Sim must run on KrakenBot"
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        pass
