from typing import Tuple
from gps.gps import _GPS
import serial.tools.list_ports as port_list
from gps.ZEDF9P import ZEDF9P
from urc_intelsys_2024_msgs.msg import GPS
import rclpy


class ActualGPSCompass(_GPS):
    def __init__(self, port: str = None) -> None:
        super().__init__()
        self.gpsState = True  # Keeps track of the reading state of the GPS
        self.cur_gps: Tuple[float, float] = None  # (longitude, latitude)
        port_number = 0
        ports = list(
            filter(lambda port: "USB" not in port.device, port_list.comports())
        )
        if port in ports:
            self.gps = ZEDF9P(port, 57600)
        else:
            print("====> No port specified. Using Port:", ports[port_number].device)
            port = ports[port_number].device
            self.gps = ZEDF9P(port, 57600)
            while not self.gps.has_gps_lock() and port_number < len(ports):
                print(f"Port {port} not working, going to next port...")
                port_number += 1
                port = ports[port_number].device
                try:
                    self.gps = ZEDF9P(port, 57600)
                    break
                except:
                    continue
        if not self.gps.has_gps_lock():
            raise Exception("Unable to get proper GPS lock.")

    def get_cur_gps(self) -> GPS:
        """
        Returns latest GPS coordinates
        """
        # cur_gps is in the form (longitude, latitude)
        return self.gps.get_position()


def main():
    rclpy.init()

    gps = ActualGPSCompass()
    rclpy.spin(gps)

    rclpy.shutdown()
