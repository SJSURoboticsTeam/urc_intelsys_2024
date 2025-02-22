from abc import ABC, abstractmethod
from datetime import datetime

class GPS(ABC):
    @abstractmethod
    def read_gps_device(self) -> tuple[tuple[float, float], tuple[float, float], datetime]:
        """
        Read the GPS device and return a three tuple of 
            Coordinates: (latitude, longitude)
            Velocity: (Speed, heading)
            UTCdatetime: python datetime
        """
    @property
    def coordinates(self):
        return self.read_gps_device()[0]
    @property
    def velocity(self):
        return self.read_gps_device()[1]
    @property
    def datetime(self):
        return self.read_gps_device()[2]
