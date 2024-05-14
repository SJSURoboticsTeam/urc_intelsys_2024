from typing import Tuple
from urc_intelsys_2024.sensors.gps.gps import _GPS
import serial.tools.list_ports as port_list
from urc_intelsys_2024.sensors.gps.ZEDF9P import ZEDF9P
from threading import Thread
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
            while self.gps.get_position() == [
                "error"
            ] * 2 or self.gps.get_position() == ["None", "None"]:
                print(f"Port {port} not working, going to next port...")
                port_number += 1
                port = ports[port_number].device
                try:
                    self.gps = ZEDF9P(port, 57600)
                    break
                except:
                    continue
        self.cur_gps = self.gps.get_position()
        if self.cur_gps is None:
            raise Exception("Unable to get proper GPS lock.")

        self.gpsThreadCall = Thread(target=self.read)

    def read(self) -> None:
        """
        On thread for reading gps coordinates
        This way we can continously retrieve the latest data
        """
        while self.gpsState:
            temp = self.gps.get_position()
            if temp is not None:
                self.cur_gps = temp

    def get_cur_gps(self) -> GPS:
        """
        Returns latest GPS coordinates
        """
        # cur_gps is in the form (longitude, latitude)
        return GPS(longitude=self.cur_gps[0], latitude=self.cur_gps[1])

    def start_service(self) -> None:
        """
        Should be called to start the service.
        """
        self.gpsThreadCall.start()

    def stop_service(self) -> None:
        """
        Should be called when we disconnect
        """
        self.gpsState = False
        self.gpsThreadCall.join()


def main():
    rclpy.init()

    gps = ActualGPSCompass()
    gps.start_service()
    rclpy.spin(gps)

    gps.stop_service()

    rclpy.shutdown()
