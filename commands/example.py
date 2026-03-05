import commands2


class ExampleCommand(commands2.Command):
    # pass in parent subsystem
    def __init__(self):
        super().__init__()
        # make sure to add requirements to parent subsystem here

    # runs every scheduled tick (think of it as a while true)
    def execute(self) -> None:
        pass

    # boolean condition to check if the command is finished (needed for running commands in series)
    def isFinished(self) -> bool:
        return False

    # code that runs after the command is finished
    def end(self, interrupted: bool) -> None:
        pass
