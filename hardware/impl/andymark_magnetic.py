from hardware.base.switch import LimitSwitch
from hardware.impl.generic_can import GenericCAN

ANDYMARK_MAGNETIC_API_ID = 32
MANUFACTURER_ID = 15
DEVICE_TYPE_ID = 10


class AndymarkMagnetic(LimitSwitch):
    def __init__(self, device_id: int) -> None:
        super().__init__()
        self.device = GenericCAN(device_id, MANUFACTURER_ID, DEVICE_TYPE_ID)
        self.device_id = device_id

    def get_state(self) -> bool:
        _health, status = self.device.get_newest_data(ANDYMARK_MAGNETIC_API_ID)

        # data[0] returns false when the limit swtich doesn't return packets after multiple calls
        # if not health:
        #     # print("Not receiving packet from limit switch " + str(self.device_id))
        # # else:
        # #     if self.device_id == 19:
        # #         # print("Got signal")

        b = status.data
        state = (b[0] & 0xFF) != 0

        return state
