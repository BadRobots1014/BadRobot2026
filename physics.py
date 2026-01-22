
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

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from pyfrc.physics.units import units

import typing

if typing.TYPE_CHECKING:
  from robot import Robot

class PhysicsEngine:
  def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
    self.physics_controller = physics_controller

  def update_sim(self, now: float, tm_diff: float) -> None:
    pass
