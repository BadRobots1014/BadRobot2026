from wpilib import AddressableLED, LEDPattern, Color

from hardware.base.ledcontroller import LEDController

import logging


class PWMLED(LEDController):

    def __init__(self, port: int, length: int):
        super().__init__()
        self.controller = AddressableLED(port)
        self.buffer = [AddressableLED.LEDData()] * length
        self.controller.setLength(length)

        self.spacing = 0.015
        self.apply_pattern(self.get_solid(255, 255, 0))
        self.controller.start()

        logging.info("Lights initialized")

    def get_solid(self, r: int, g: int, b: int):
        return LEDPattern.solid(Color(r, g, b))

    def get_rainbow(self, saturation: int, value: int, speed: float):
        return LEDPattern.rainbow(saturation, value).scrollAtAbsoluteSpeed(
            speed, self.spacing
        )

    def get_gradient(self, continuous: bool, colors: list[tuple]):
        return LEDPattern.gradient(
            (
                LEDPattern.GradientType.kContinuous
                if continuous
                else LEDPattern.GradientType.kDiscontinuous
            ),
            [Color(color[0], color[1], color[2]) for color in colors],
        )

    def apply_pattern(self, pattern: LEDPattern):
        pattern.applyTo(self.buffer)
        self.controller.setData(self.buffer)
