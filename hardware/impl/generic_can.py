import wpilib


class GenericCAN:
    def __init__(
        self, device_id: int, manufacturer_id: int, device_type_id: int
    ) -> None:
        self.device = wpilib.CAN(device_id, manufacturer_id, device_type_id)

    def get_latest_data(self, api_id: int) -> tuple[bool, wpilib.CANData]:
        """
        Returns the most recently received CAN packet.

        :param: The API ID to read.
        :returns: isValid and latest CANData
        """
        data = wpilib.CANData()
        is_valid = self.device.readPacketLatest(api_id, data)

        return is_valid, data

    def get_newest_data(self, api_id: int) -> tuple[bool, wpilib.CANData]:
        """
        This will only return properly once per packet received.
        Multiple calls without receiving another packet will return false.

        :param: The API ID to read.
        :returns: isValid and latest CANData
        """
        data = wpilib.CANData()
        is_valid = self.device.readPacketNew(api_id, data)

        return is_valid, data

    def get_data_with_timeout(
        self, timeout_ms: int, api_id: int
    ) -> tuple[bool, wpilib.CANData]:
        data = wpilib.CANData()
        is_valid = self.device.readPacketTimeout(api_id, timeout_ms, data)

        return is_valid, data
