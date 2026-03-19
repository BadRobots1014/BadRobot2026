from enum import Enum

from commands2 import Subsystem
import wpilib


class PiLights(Subsystem):
    def __init__(self):
        super().__init__()
        self.high = False

        self.PWMOUT = wpilib.DigitalOutput(0)

    def set_state(self, state: LEDState) -> None:
        self.PWMOUT.pulse(state.value)
        print(f"Changing state to {state.name} - {state.value}")


class LEDState(Enum):
    HOPPER_EXTEND = 0.00002  # 1
    HOPPER_RETRACT = 0.00004  # 2
    HOPPER_EXTENDED = 0.00006  # 3
    HOPPER_RETRACTED = 0.00008  # 4
    AUTO = 0.0001  # 5
    RADIUS = 0.00012  # 6
    SLOW_MODE = 0.00014
    SHOOTER_REV = 0.00016
    SHOOTER_READY = 0.00018
    PARTY_MODE = -1  # TODO Add rainbow pattern
