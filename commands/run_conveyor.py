from commands2 import Command

from subsystems.conveyor import ConveyorSubsystem


class RunConveyor(Command):
    def __init__(self, conveyor: ConveyorSubsystem, shoot_direction: bool):
        super().__init__()
        self.addRequirements(conveyor)
        self.conveyor = conveyor
        self.shoot_direction = shoot_direction

    def execute(self) -> None:
        if self.shoot_direction:
            self.conveyor.set_conveyor_shoot_voltage_from_networktable()
        else:
            self.conveyor.set_conveyor_dump_voltage_from_networktable()

    def end(self, interrupted: bool) -> None:
        self.conveyor.set_conveyor_voltage(0)
