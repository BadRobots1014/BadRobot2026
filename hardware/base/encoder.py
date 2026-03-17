from abc import ABC, abstractmethod


class Encoder(ABC):
    @abstractmethod
    def get_velocity(self) -> float: ...

    @abstractmethod
    def get_position(self) -> float: ...

    @abstractmethod
    def set_position(self, position: float) -> None: ...
