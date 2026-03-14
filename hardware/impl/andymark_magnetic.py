from hardware.base.switch import LimitSwitch
from hardware.impl.generic_can import GenericCAN

ANDYMARK_MAGNETIC_API_ID = 32
MANUFACTURER_ID = 15
DEVICE_TYPE_ID = 10


class AndymarkMagnetic(LimitSwitch):
    def __init__(self, device_id: int) -> None:
        super().__init__()
        self.device = GenericCAN(device_id, MANUFACTURER_ID, DEVICE_TYPE_ID)

    def get_state(self) -> bool:
        data = self.device.get_latest_data(ANDYMARK_MAGNETIC_API_ID)
        b = data[1].data
        state = (b[0] & 0xFF) != 0
        return state
