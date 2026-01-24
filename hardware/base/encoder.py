from abc import ABC

from wpiutil import Sendable

from hardware.base import SendableABCMeta


class Encoder(Sendable, ABC, metaclass = SendableABCMeta):

    def get_velocity(self):
        pass

    def get_position(self):
        pass

    def set_position(self, position: float):
        pass