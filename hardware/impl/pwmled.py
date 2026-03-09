import typing

from wpilib import AddressableLED, Color, LEDPattern

from hardware.base.ledcontroller import LEDController


class PWMLED(LEDController):
    def __init__(self, port: int, length: int):
        super().__init__()
        self.controller = AddressableLED(port)
        self.buffer = [AddressableLED.LEDData() for _ in range(length)]
        self.controller.setLength(length)

        self.spacing = 0.015
        self.apply_pattern(self.get_solid(0, 0, 0))
        self.controller.start()

        print("Lights initialized")

    def get_solid(self, r: int, g: int, b: int) -> LEDPattern:
        return LEDPattern.solid(Color(r, g, b))

    def get_rainbow(self, saturation: int, value: int, speed: float) -> LEDPattern:
        return LEDPattern.rainbow(saturation, value).scrollAtAbsoluteSpeed(
            speed, self.spacing
        )

    def get_gradient(self, continuous: bool, colors: list[tuple]) -> LEDPattern:
        return LEDPattern.gradient(
            (
                LEDPattern.GradientType.kContinuous
                if continuous
                else LEDPattern.GradientType.kDiscontinuous
            ),
            [Color(color[0], color[1], color[2]) for color in colors],
        )

    def led_writter(
        self, led_index: typing.SupportsInt | typing.SupportsIndex, color: Color
    ) -> None:
        self.buffer[led_index].setLED(color)

    def apply_pattern(self, pattern: LEDPattern) -> None:
        pattern.applyTo(self.buffer, self.led_writter)
        self.controller.setData(self.buffer)
