import commands2.subsystem


class Example(commands2.Subsystem):
    def __init__(self):
        super().__init__()

    # extra methods here

    # runs every scheduled tick
    def periodic(self):
        pass
