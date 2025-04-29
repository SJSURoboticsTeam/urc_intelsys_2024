from datetime import datetime
from .GPS import GPS
import serial
from serial.tools.list_ports import comports
import re
import time

class DeviceNotFoundException(Exception):
    """Exception to indicate device was not found"""

# device_description = 'u-blox GNSS receiver'
class ZED_F9P(GPS):
    def __init__(self, port=None) -> None:
        super().__init__()
        if port is None:
            port = self.find_device('u-blox GNSS receiver')
        self.serial = serial.Serial(port)
        self.pattern = re.compile(r"\$GNRMC,(\d+\.\d+)?,([AV])?,(\d+\.\d+)?,([NS])?,(\d+\.\d+)?,([EW])?,(\d+\.\d+)?,(\d+\.\d+)?(\d{6})?")
    def find_device(self, device_description):
        """
        Find the gps device or throw an exception if its not found
        """
        for i in comports():
            if i.description == device_description:
                return i.device
        raise DeviceNotFoundException("GPS device was not found")
    def read_gps_device(self, attempts=3, reattempt_wait=0.1) -> tuple[tuple[float, float], tuple[float, float], datetime]:
        def read():
            for attempt in range(attempts):
                try:
                    return self.pattern.findall(self.serial.read_all().decode('utf-8'))[-1]
                except IndexError:
                    pass
                time.sleep(reattempt_wait)
            raise Exception("Got no GPS reading")
        utc, status, lattitude, ns, longitude, ew, speed, heading, date = read()
        if status != 'A': raise Exception("No lock yet")
        coords = None if '' in [lattitude, longitude] else (
            float(lattitude) * (1 if ns=='N' else -1),
            float(longitude) * (1 if ew=='E' else -1)            
        )
        velocity = None if '' in [speed, heading] else (
            float(speed),
            float(heading)
        )
        utc_datetime = None if '' in [utc, date] else time.strptime(utc+date, r'%d%m%y%H%M%S.%f')
        return (
            coords,
            velocity,
            utc_datetime
        )
        
            
        
        
    

    