#!/usr/bin/env python3

import logging
import struct
import time
from typing import List, Optional

from .ry_hand_interface import (
    CommunicationInterface,
    RuiyanFingerControlMessage,
    RuiyanInstructionType,
    RuiyanStatusCode,
    RuiyanFingerStatusMessage,
)

logger = logging.getLogger(__name__)


class RuiyanHandController:

    def __init__(
        self,
        communication_interface: CommunicationInterface,
        motors_id: [int],
        instruction: RuiyanInstructionType = None,
    ):

        self.motor_ids = motors_id
        self.communication_interface = communication_interface
        self.instruction = instruction
        self.position_list = [0, 0, 0, 0, 0, 0]
        self.velocity_list = [0, 0, 0, 0, 0, 0]
        self.current_list = [0, 0, 0, 0, 0, 0]
        self.command_names = {
            0xA0: "Read motor information",
            0xAA: "Position velocity current hybrid control",
            0xA5: "Clear motor error",
        }

    def connect(self) -> bool:
        return self.communication_interface.connect()

    def disconnect(self):
        return self.communication_interface.disconnect()

    def is_connected(self) -> bool:
        return self.communication_interface.is_connected()

    def _validate_motor_ids(self, motor_ids: List[int]) -> bool:
        for motor_id in motor_ids:
            if motor_id not in self.motor_ids:
                return False
        return True

    def _set_motor(
        self,
        motor_id: int,
        position: Optional[int],
        velocity: Optional[int],
        current: Optional[int],
    ) -> bool:
        request_message = RuiyanFingerControlMessage(
            motor_id=motor_id,
            instruction=self.instruction,
            position=position,
            velocity=velocity,
            current=current,
        )
        logger.debug(
            f"Motor ID: {request_message.motor_id}, "
            f"Instruction: {request_message.instruction}, "
            f"Position: {request_message.position}, "
            f"Velocity: {request_message.velocity}, "
            f"Current: {request_message.current}"
        )
        return self.communication_interface.send_message(request_message)
        # status = self._parse_response(
        #     self.communication_interface.send_and_receive(
        #         message=request_message
        #     )
        # )
        # return status
        return True

    def set(
        self,
        position_list: Optional[List[int]] = None,
        velocity_list: Optional[List[int]] = None,
        current_list: Optional[List[int]] = None,
    ) -> bool:
        if position_list is not None:
            self.position_list = position_list
        if velocity_list is not None:
            self.velocity_list = velocity_list
        if current_list is not None:
            self.current_list = current_list
        return True

    def loop(self) -> List[RuiyanFingerStatusMessage]:
        status_list = []
        
        # Clear serial port buffer before sending commands
        self.communication_interface.clear_buffer()
        
        for index, motor_id in enumerate(self.motor_ids):
            self._set_motor(
                motor_id=motor_id,
                position=self.position_list[index],
                velocity=self.velocity_list[index],
                current=self.current_list[index],
            )
            time.sleep(0.001)

        received_bytes = self.communication_interface.receive_bytes()
        time.sleep(0.001)
        
        # Check if received data is valid, if not, skip this publish cycle
        if received_bytes is None or len(received_bytes) != 13 * 6:
            logger.warning(f"Received invalid data length: {len(received_bytes) if received_bytes else 'None'}, expected {13 * 6}. Skipping this publish cycle.")
            return status_list  # Return empty list instead of crashing
        
        bytes_list = [received_bytes[i*13:(i+1)*13] for i in range(6)]
        for raw_bytes in bytes_list:
            status_list.append(self._parse_response(raw_bytes))
        return status_list

    def _parse_response(self, raw_bytes:bytes) -> RuiyanFingerStatusMessage:
        if len(raw_bytes) < 13:
            logger.warning(f"Received insufficient data: {len(raw_bytes)} bytes, expected at least 13")
            return None

        header, motor_id, _, data_length = struct.unpack("<4B", raw_bytes[:4])

        if header != 0xA5:
            logger.error(
                f"Header error: expected 0xA5, received 0x{header:02X}"
            )
            return None

        finger_data = raw_bytes[4:12]

        data_uint64 = struct.unpack("<Q", finger_data)[0]

        instruction = (data_uint64 >> 0) & 0xFF
        status = (data_uint64 >> 8) & 0xFF
        position = (data_uint64 >> 16) & 0xFFF
        velocity = (data_uint64 >> 28) & 0xFFF
        current = (data_uint64 >> 40) & 0xFFF

        if velocity & 0x800:
            velocity = velocity - 0x1000
        if current & 0x800:
            current = current - 0x1000

        try:
            instruction_type = RuiyanInstructionType(instruction)
        except ValueError:
            logger.error(f"Invalid instruction type: {instruction} (0x{instruction:02X}), expected one of: 0xA0, 0xAA, 0xA5")
            return None

        response_message = RuiyanFingerStatusMessage(
            motor_id=motor_id,
            instruction=instruction_type,
            status=status,
            position=position,
            velocity=velocity,
            current=current,
        )

        if status != 0:
            status_desc = RuiyanStatusCode.get_description(status)
            logger.error(
                f"Error - Motor ID: {motor_id}, "
                f"Status code: {status}, "
                f"Error message: {status_desc}"
            )
        else:
            logger.debug(
                f"Received - Motor ID: {response_message.motor_id}, "
                f"Instruction: {response_message.instruction}, "
                f"Status: {response_message.status}, "
                f"Position: {response_message.position}, "
                f"Velocity: {response_message.velocity}, "
                f"Current: {response_message.current}"
            )
        return response_message

