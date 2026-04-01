from __future__ import annotations

import json
from pathlib import Path
from typing import ClassVar

from commands2.button import CommandGenericHID, Trigger
from ntcore import NetworkTableInstance
import wpilib

button_to_string: dict[int, str] = {
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

# Where bindings are written so generate_controller_map.py can read them
# without needing to import anything from the RobotPy environment.
# Path is relative to this file: ../controller_bindings.json
_BINDINGS_FILE: Path = (
    Path(__file__).resolve().parent / ".." / "controller_bindings.json"
)


def write_binds() -> None:
    # Only write to disk on dev machines — never on the roboRIO
    if not wpilib.RobotBase.isReal():
        CustomController.flush_all()


class CustomController(CommandGenericHID):
    # All instances keyed by port so _flush_all writes one tidy JSON file
    _registry: ClassVar[dict[int, CustomController]] = {}

    def __init__(self, port: int) -> None:
        super().__init__(port)
        self.port = port
        self.button_publishers: list = []
        self._inst = NetworkTableInstance.getDefault()
        self._controller_table = self._inst.getTable("CustomController" + str(port))
        self._bindings: dict[str, str] = {}
        CustomController._registry[port] = self

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def add_pub(self, button: str, command_name: str) -> None:
        topic = self._controller_table.getStringTopic(button)
        pub = topic.publish()
        pub.set(command_name)
        self.button_publishers.append(pub)

        self._bindings[button] = command_name

    def create_button(self, button: int, command_name: str) -> Trigger:
        self.add_pub(button_to_string[button], command_name)
        return self.button(button)

    def bind_pov_up(self, name: str) -> Trigger:
        self.add_pub("POVUP", name)
        return self.povUp()

    def bind_pov_down(self, name: str) -> Trigger:
        self.add_pub("POVDOWN", name)
        return self.povDown()

    def bind_pov_left(self, name: str) -> Trigger:
        self.add_pub("POVLEFT", name)
        return self.povLeft()

    def bind_pov_right(self, name: str) -> Trigger:
        self.add_pub("POVRIGHT", name)
        return self.povRight()

    # ------------------------------------------------------------------
    # JSON serialisation — only called on dev machines
    # ------------------------------------------------------------------

    @classmethod
    def flush_all(cls) -> None:
        """Write all registered controllers' bindings to controller_bindings.json."""
        data = {
            str(port): ctrl._bindings
            for port, ctrl in sorted(cls._registry.items())
            if port in (0, 1)
        }
        _BINDINGS_FILE.parent.mkdir(parents=True, exist_ok=True)
        _BINDINGS_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
