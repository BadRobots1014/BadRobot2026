from abc import ABC, abstractmethod

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class LimitSwitch(Sendable, ABC, metaclass=SendableABCMeta):
    @abstractmethod
    def get_state(self) -> bool: ...
