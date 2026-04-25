import commands2
from subsystems.swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
import kraken_container


class DriveForward(commands2.Command):
    def __init__(self, drive_train: CommandSwerveDrivetrain):
        super().__init__()
        self.drive_train = drive_train
        self.addRequirements(drive_train)

        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(kraken_container.DRIVE_DEADBAND)
            .with_rotational_deadband(kraken_container.ANGULAR_DEADBAND)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

        self.drive_train.set_control(self._drive.with_velocity_x(1.5))

    def end(self, interrupted: bool) -> None:
        self.drive_train.set_control(self._drive.with_velocity_x(0))