import commands2

from subsystems.shooter import Shooter



class Shoot(commands2.Command):
    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = Shooter

    def execute(self):
            
