from commands2 import ParallelCommandGroup
from pathplannerlib.auto import NamedCommands

from commands.extend_hopper import ExtendHopperCommand
from commands.run_intake import RunIntakeCommand
from robot_class import RobotClass
from routines.auto_shoot_with_intake import AutoShootWithIntake
from routines.shoot_when_ready import ShootWhenReady


def register_commands(robot: RobotClass) -> None:
    """
    Register named commands for path planner
    """
    NamedCommands.registerCommand(
        "Extend",
        ExtendHopperCommand(robot.hopper).withTimeout(1),
    )

    NamedCommands.registerCommand(
        "Slight Dump",
        RunIntakeCommand(robot.intake, dump=True).withTimeout(0.1),
    )

    NamedCommands.registerCommand(
        "RunIntake",
        RunIntakeCommand(robot.intake, dump=False).withTimeout(6),
    )

    NamedCommands.registerCommand(
        "ShootStarting8",
        ShootWhenReady(
            robot.shooter,
            robot.kicker,
            robot.conveyor,
            3500,
        ).withTimeout(3),
    )

    NamedCommands.registerCommand(
        "EmptyHopper",
        ParallelCommandGroup(
            ShootWhenReady(
                robot.shooter,
                robot.kicker,
                robot.conveyor,
                3500,
            ),
            AutoShootWithIntake(robot.intake),
        ).withTimeout(4),
    )
