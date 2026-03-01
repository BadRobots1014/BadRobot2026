import wpilib


class GenericCAN:
    def __init__(self, deviceId: int, manufacturerId: int, deviceTypeId: int) -> None:
        self.device = wpilib.CAN(deviceId, manufacturerId, deviceTypeId)

    def get_latest_data(self, api_id: int) -> tuple[bool, wpilib.CANData]:
        """
        Returns the most recently received CAN packet.

        :param: The API ID to read.
        :returns: isValid and latest CANData
        """
        data = wpilib.CANData()
        isValid = self.device.readPacketLatest(api_id, data)

        return isValid, data

    def get_newest_data(self, api_id: int) -> tuple[bool, wpilib.CANData]:
        """
        This will only return properly once per packet received.
        Multiple calls without receiving another packet will return false.

        :param: The API ID to read.
        :returns: isValid and latest CANData
        """
        data = wpilib.CANData()
        isValid = self.device.readPacketNew(api_id, data)

        return isValid, data
