import rev

from hardware.base.encoder import Encoder


class SparkRelativeEncoder(Encoder):
    def __init__(self, rev_encoder: rev.SparkRelativeEncoder):
        super().__init__()
        self.encoder = rev_encoder

    def get_velocity(self) -> float:
        return self.encoder.getVelocity()

    def get_position(self) -> float:
        return self.encoder.getPosition()

    def set_position(self, position: float) -> None:
        self.encoder.setPosition(position)
