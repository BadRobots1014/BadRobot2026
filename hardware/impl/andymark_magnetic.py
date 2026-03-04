from hardware.impl.generic_can import GenericCAN
from hardware.base.switch import LimitSwitch

AndymarkMagneticApiId = 32


class AndymarkMagnetic(LimitSwitch):
    def __init__(self, deviceId: int) -> None:
        manufacturerId = 15
        deviceTypeId = 10
        self.device = GenericCAN(deviceId, manufacturerId, deviceTypeId)

    def get_state(self) -> bool:
        data = self.device.get_latest_data(AndymarkMagneticApiId)
        return True if data[0] == 1 else False
