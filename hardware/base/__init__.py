from abc import ABCMeta

from wpiutil import Sendable


class SendableABCMeta(ABCMeta, type(Sendable)):
    pass