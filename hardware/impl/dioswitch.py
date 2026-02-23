from wpilib import DigitalInput

from hardware.base.switch import LimitSwitch


class DIOSwitch(LimitSwitch):
    def __init__(self, port: int):
        super().__init__()
        self.switch = DigitalInput(port)

    def get_state(self) -> bool:
        return self.switch.get()
