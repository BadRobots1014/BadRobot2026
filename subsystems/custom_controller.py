from commands2.button import CommandGenericHID, Trigger
from ntcore import NetworkTableInstance

button_to_string = {
    0: "UNDEFINED",
    1: "SQUARE",
    2: "CROSS",
    3: "CIRCLE",
    4: "TRIANGLE",
    5: "L1",
    6: "R1",
    7: "L2",
    8: "R2",
    9: "SHARE",
}


class CustomController(CommandGenericHID):
    bounded_buttons: list[str]

    def __init__(self, port: int):
        super().__init__(port)
        self.bounded_buttons = []
        self._inst = NetworkTableInstance.getDefault()
        self._controller_table = self._inst.getTable("CustomController" + str(port))
        self._bounded_buttons_publisher = self._controller_table.getStringArrayTopic(
            "BoundedButtons"
        ).publish()

    def create_button(self, button: int, command_name: str) -> Trigger:

        self.bounded_buttons.append(button_to_string[button] + " -> " + command_name)
        self._bounded_buttons_publisher.set(self.bounded_buttons)

        return self.button(button)
