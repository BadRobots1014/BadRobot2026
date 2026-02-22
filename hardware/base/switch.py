from abc import ABC

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class LimitSwitch(Sendable, ABC, metaclass=SendableABCMeta):
    def get_state(self) -> bool:
        pass
