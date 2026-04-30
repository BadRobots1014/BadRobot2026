from commands2 import Command, InstantCommand, WaitCommand
from phoenix6 import controls
from wpimath._controls._controls.trajectory import TrapezoidProfile

from subsystems.hopper import HopperSubsystem

MAX_VELOCITY = 120
MAX_ACCELERATION = 480


class ExtendHopperCommand(Command):
    def __init__(
        self,
        hopper: HopperSubsystem,
    ):
        """
        Use Network Tables to Extend / Retract hopper.

        :param extend: Whether to extend or retract hopper
        """
        super().__init__()
        self.hopper = hopper

        self.finished = False
        self.waiting = False

        self.initial_pos = self.hopper.get_extension_position()
        self.profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
        )
        self.goal = TrapezoidProfile.State(self.hopper.get_max_extension_value(), 0)

        self.request = controls.PositionVoltage(0).with_slot(0)
        self.setpoint = TrapezoidProfile.State()

        self.addRequirements(hopper)

    def execute(self) -> None:
        self.setpoint = self.profile.calculate(0.02, self.setpoint, self.goal)

        # self.request.position = self.setpoint.position
        # self.request.velocity = self.setpoint.velocity
        # self.hopper.set_extension_position_and_velocity(self.request)
        self.hopper.set_extension_voltage_from_networktable()

    def setFinished(self) -> None:
        self.finished = True

    def isFinished(self) -> bool:
        # Finish on limit
        if self.finished:
            return True
        if self.hopper.is_forward_extended() and not self.waiting:
            # Schedule a command then check its status on every isFinished to see if it has completed
            WaitCommand(0.05).andThen(InstantCommand(self.setFinished)).schedule()
            self.waiting = True
        return False

    def end(self, interrupted: bool) -> None:
        self.hopper.has_hopper_extended = True
        self.hopper.set_extension_voltage(0)
