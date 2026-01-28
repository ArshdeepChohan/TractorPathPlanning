"""
GPS Reader for Real Hardware (Placeholder).
Intended to read NMEA data from Serial/UART.
"""
from src.gps_interface import GPSInterface
# import serial
# import pynmea2

class GPSReader(GPSInterface):
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        # self.serial_conn = serial.Serial(port, baudrate, timeout=1)
        print(f"GPS Reader initialized on {port} (Hardware Mode)")

    def update(self, steering_angle, dt):
        # In real hardware mode, 'update' does nothing to the vehicle state 
        # because the vehicle moves physically.
        pass

    def get_state(self):
        """
        Read from serial port and parse NMEA sentences.
        """
        # Placeholder implementation
        # line = self.serial_conn.readline().decode('ascii', errors='replace')
        # msg = pynmea2.parse(line)
        # return parsed lat/lon/heading
        
        raise NotImplementedError("Hardware GPS reading not implemented yet.")
