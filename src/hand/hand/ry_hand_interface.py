#!/usr/bin/env python3

import logging
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional

logger = logging.getLogger(__name__)


class RuiyanInstructionType(IntEnum):
    READ_MOTOR_INFO = 0xA0
    CTRL_MOTOR_POSITION_VELOCITY_CURRENT = 0xAA
    CLEAR_MOTOR_ERROR = 0xA5


class RuiyanStatusCode(IntEnum):
    SERVO_OK = 0
    SERVO_TEMPERATURE_HIGH_WARN = 1
    SERVO_TEMPERATURE_HIGH_ERROR = 2
    SERVO_VOLTAGE_LOW_ERROR = 3
    SERVO_VOLTAGE_HIGH_ERROR = 4
    SERVO_CURRENT_OVER_ERROR = 5
    SERVO_TORQUE_OVER_ERROR = 6
    SERVO_FUSE_ERROR = 7
    SERVO_PWM_ERROR = 8
    SERVO_DRIVE_ERROR = 9
    SERVO_HALL_ERROR = 10
    SERVO_FAIL = 250
    SERVO_PARAM_ERROR = 251
    SERVO_LIB_INIT_ERROR = 252
    CAN_FORMAT_ERROR = 253
    CAN_MSG_SENT_FAIL = 254
    LIB_HOOK_APPLY_FAILED = 255

    @classmethod
    def get_description(cls, status_code: int) -> str:
        descriptions = {
            cls.SERVO_OK: "Operation successful",
            cls.SERVO_TEMPERATURE_HIGH_WARN: "Motor temperature high warning",
            cls.SERVO_TEMPERATURE_HIGH_ERROR: "Motor temperature high protection",
            cls.SERVO_VOLTAGE_LOW_ERROR: "Motor low voltage protection",
            cls.SERVO_VOLTAGE_HIGH_ERROR: "Motor high voltage protection",
            cls.SERVO_CURRENT_OVER_ERROR: "Motor overcurrent protection",
            cls.SERVO_TORQUE_OVER_ERROR: "Motor over torque protection",
            cls.SERVO_FUSE_ERROR: "Motor fuse error protection",
            cls.SERVO_PWM_ERROR: "Motor stall protection",
            cls.SERVO_DRIVE_ERROR: "Driver exception protection",
            cls.SERVO_HALL_ERROR: "Motor hall error protection",
            cls.SERVO_FAIL: "Servo no response or timeout",
            cls.SERVO_PARAM_ERROR: "Servo response data error",
            cls.SERVO_LIB_INIT_ERROR: "Servo group structure not initialized",
            cls.CAN_FORMAT_ERROR: "CAN data format error",
            cls.CAN_MSG_SENT_FAIL: "CAN message send failed",
            cls.LIB_HOOK_APPLY_FAILED: "Hook application failed",
        }
        return descriptions.get(
            status_code, f"Unknown status code: {status_code}"
        )




@dataclass
class RuiyanFingerControlMessage:
    motor_id: int
    instruction: RuiyanInstructionType
    position: Optional[int]
    velocity: Optional[int]
    current: Optional[int]

    def print(self):
        print(
            f"motor_id: {self.motor_id}, "
            f"instruction: {self.instruction}, "
            f"position: {self.position}, "
            f"velocity: {self.velocity}, "
            f"current: {self.current}"
        )


@dataclass
class RuiyanFingerStatusMessage:
    motor_id: int
    instruction: RuiyanInstructionType
    status: int
    position: Optional[int]
    velocity: Optional[int]
    current: Optional[int]

    def print(self):
        print(
            f"Motor ID: {self.motor_id}, "
            f"Instruction: {self.instruction}, "
            f"Status: {self.status}, "
            f"Position: {self.position}, "
            f"Velocity: {self.velocity}, "
            f"Current: {self.current}"
        )


class CommunicationInterface(ABC):

    def __init__(self, auto_connect: bool):
        self.connected = False
        if auto_connect:
            self.connect()

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @abstractmethod
    def send_message(self, message: RuiyanFingerControlMessage) -> bool:
        pass

    @abstractmethod
    def receive_bytes(self) -> bytes:
        pass


class SerialInterface(CommunicationInterface):

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float = 0.015,
        auto_connect: bool = False,
        mock: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_controller = None
        self.mock = mock
        super().__init__(auto_connect)

    def connect(self) -> bool:
        if self.mock:
            self.connected = True
            return True
        try:
            import serial

            self.serial_controller = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            self.connected = True
            logger.info(f"Serial port opened successfully: {self.port}")
            return True
        except ImportError:
            self.connected = False
            logger.error(
                "Serial library not installed, please install pyserial: "
                "pip install pyserial"
            )
            return False
        except Exception as e:
            self.connected = False
            logger.error(f"Serial port open failed: {e}")
            return False

    def disconnect(self):
        if self.serial_controller and self.connected:
            self.serial_controller.close()
            self.connected = False
            logger.info("Serial connection disconnected")

    def is_connected(self) -> bool:
        return self.connected

    def send_message(self, message: RuiyanFingerControlMessage) -> bool:
        if not self.connected:
            logger.error("Serial port not connected")
            return False

        if self.mock:
            serial_frame = self._build_serial_frame(message)
            logger.debug(
                f"Send - Motor ID: {message.motor_id}, "
                f"Instruction: {hex(message.instruction)}, "
                f"Frame data: {' '.join([f'{byte:02X}' for byte in serial_frame])}"  # noqa E999
            )
            return True

        try:
            serial_frame = self._build_serial_frame(message)
            self.serial_controller.write(serial_frame)
            logger.debug(
                f"Send - Motor ID: {message.motor_id}, "
                f"Instruction: {hex(message.instruction)}, "
                f"Frame data: {' '.join([f'{byte:02X}' for byte in serial_frame])}"  # noqa
            )
            return True
        except Exception as e:
            logger.error(f"Serial message send failed: {e}")
            return False

    def receive_bytes(self) -> bytes:
        if not self.connected:
            return None
        try:
            response = self.serial_controller.read(13*6)
            return response
        except Exception as e:
            logger.error(f"Serial message receive failed: {e}")
            return None

    def clear_buffer(self) -> bool:
        """Clear the serial port input and output buffers."""
        if not self.connected:
            logger.warning("Serial port not connected, cannot clear buffer")
            return False
        
        if self.mock:
            logger.debug("Mock mode, buffer clear simulated")
            return True
            
        try:
            # Clear input buffer (received data)
            self.serial_controller.reset_input_buffer()
            # Clear output buffer (data to be sent)
            self.serial_controller.reset_output_buffer()
            logger.debug("Serial port buffers cleared successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to clear serial port buffers: {e}")
            return False

    def _build_serial_frame(self, message: RuiyanFingerControlMessage) -> bool:
        # Clamp values into the ushort range (0-65535).
        original_position = message.position or 0
        original_velocity = message.velocity or 0
        original_current = message.current or 0

        position = max(0, min(65535, original_position))
        velocity = max(0, min(65535, original_velocity))
        current = max(0, min(65535, original_current))

        if original_position != position:
            logger.warning(f"Position value {original_position} clamped to {position} (ushort range: 0-65535)")
        if original_velocity != velocity:
            logger.warning(f"Velocity value {original_velocity} clamped to {velocity} (ushort range: 0-65535)")
        if original_current != current:
            logger.warning(f"Current value {original_current} clamped to {current} (ushort range: 0-65535)")
        
        serial_frame = struct.pack(
            "<B B B 2B 3H 1B",
            0xA5,
            message.motor_id,
            0x00,
            0x08,
            message.instruction,
            position,
            velocity,
            current,
            0x00,
        )

        checksum = 0
        for byte in serial_frame:
            checksum += byte

        serial_frame = struct.pack(
            "<B B B 2B 3H 1B 1B",
            0xA5,
            message.motor_id,
            0x00,
            0x08,
            message.instruction,
            position,
            velocity,
            current,
            0x00,
            checksum & 0xFF,
        )

        return serial_frame

    # def send(self, message: RuiyanFingerControlMessage) -> bool:
    #     if not self._send_message(message):
    #         return False
    #     return True
        # if self.mock:
        #     logger.debug("Mock mode, no need to receive data")
        #     mock_response = struct.pack(
        #         "<5B 8B",
        #         0xA5,
        #         message.motor_id,
        #         0x00,
        #         0x08,
        #         int(message.instruction),
        #         0x00,
        #         0x00,
        #         0x00,
        #         0x00,
        #         0x00,
        #         0x00,
        #         0x00,
        #         0x00,
        #     )
        #     return mock_response
        # response = self._receive_bytes()
        # return response if response is not None else b""
