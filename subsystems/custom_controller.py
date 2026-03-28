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
    10: "OPTION",
    11: "L3",
    12: "R3",
    13: "HOME",
    14: "TRACKPAD",
}


class CustomController(CommandGenericHID):
    bounded_buttons: list[str]

    def __init__(self, port: int):
        super().__init__(port)
        self.button_publishers = []
        self._inst = NetworkTableInstance.getDefault()
        self._controller_table = self._inst.getTable("CustomController" + str(port))

    def add_pub(self, button: str, command_name: str) -> None:
        topic = self._controller_table.getStringTopic(button)
        pub = topic.publish()
        pub.set(command_name)
        self.button_publishers.append(pub)

    def create_button(self, button: int, command_name: str) -> Trigger:
        self.add_pub(button_to_string[button], command_name)
        return self.button(button)

    def bind_pov_up(self, name: str) -> Trigger:
        self.add_pub("POVUP", name)
        return self.povUp()

    def bind_pov_down(self, name: str) -> Trigger:
        self.add_pub("POVDOWN", name)
        return self.povUp()

    def bind_pov_left(self, name: str) -> Trigger:
        self.add_pub("POVLEFT", name)
        return self.povUp()

    def bind_pov_right(self, name: str) -> Trigger:
        self.add_pub("POVRIGHT", name)
        return self.povUp()
