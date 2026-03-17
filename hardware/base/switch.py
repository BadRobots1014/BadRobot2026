from abc import ABC, abstractmethod


class LimitSwitch(ABC):
    @abstractmethod
    def get_state(self) -> bool: ...
