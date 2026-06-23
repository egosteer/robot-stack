import os
import struct
import time

import serial


POS_FACTOR = 0.087 * 0.01745329
RAD_TO_DEG = 180.0 / 3.14159265359


class SimpleGloveReader:
    def __init__(self, port='/dev/ttyUSB2', baudrate=500000, timeout=0.02):
        assert os.path.exists(port), f'serial port {port} does not exist'
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            return True
        except Exception as e:
            print(f'connection failed: {e}')
            return False

    def calculate_crc(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def generate_modbus_frame(self, slave_addr, func_code, start_addr, count):
        frame = struct.pack('>BBHH', slave_addr, func_code, start_addr, count)
        crc = self.calculate_crc(frame)
        return frame + crc

    def read_angles(self):
        if not self.ser or not self.ser.is_open:
            if not self.connect():
                return None
        try:
            frame = self.generate_modbus_frame(0x01, 0x03, 0x0001, 21)
            self.ser.write(frame)
            self.ser.flush()

            time.sleep(0.005)

            response = b''
            max_wait_time = 0.015
            check_interval = 0.001

            start_check = time.time()
            while time.time() - start_check < max_wait_time:
                if self.ser.in_waiting > 0:
                    response += self.ser.read(self.ser.in_waiting)
                    if len(response) >= 47:
                        break
                    time.sleep(check_interval)
                else:
                    time.sleep(check_interval)

            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)

            if not response or len(response) < 47:
                return None

            modbus_start = -1
            for i in range(len(response) - 5):
                if (
                    response[i] == 0x01
                    and response[i + 1] == 0x03
                    and response[i + 2] == 42
                    and i + 47 <= len(response)
                ):
                    modbus_start = i
                    break

            if modbus_start == -1:
                return None

            start_idx = modbus_start + 3
            data_bytes = response[start_idx:start_idx + 42]
            if len(data_bytes) < 42:
                return None

            angles = []
            for i in range(0, 42, 2):
                raw_value = (data_bytes[i] << 8) | data_bytes[i + 1]
                angle_rad = raw_value * POS_FACTOR
                angle_deg = angle_rad * RAD_TO_DEG
                angles.append(angle_deg)
            return angles
        except Exception:
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
