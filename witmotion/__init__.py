import logging

from enum import Enum
from threading import Thread

import serial

from . import protocol

log = logging.getLogger(__name__)


class ReceiveState(Enum):
    """
    Indicates the state of the receive FSM. Behavior in each state is as
    follows:

    `idle`

    - If a 0x55 is received, go to `header`.
    - If anything else is received, drop it and stay `idle`.

    `header`

    - If a valid message code is received, set the message class and go to
    `payload`.
    - If anything else is received, drop it and go to `idle`.

    `payload`

    - Block until 9 bytes is received.
    - Compute checksum.
    - If checksum is invalid, log an error.
    - If checksum is valid, parse the message and emit it.
    - Always go to `idle`.
    """

    idle = 0
    header = 1
    payload = 2


class IMU:
    """
    Main IMU interface. Instantiate to connect to a device.

    Can be used via a polling interface or a streaming callback-based
    interface.
    """

    def __init__(self, path="/dev/ttyUSB0", baudrate=9600):
        self.ser = serial.Serial(path, baudrate=baudrate, timeout=0.1)
        self.should_exit = False
        self.rxthread = Thread(target=self._rxloop)
        self.rxthread.start()

    def close(self):
        self.should_exit = True

    def _safe_read(self, size):
        buf = bytearray()
        remaining = size
        while (remaining > 0) and not self.should_exit:
            chunk = self.ser.read(size=remaining)
            buf.extend(chunk)
            remaining -= len(chunk)
        return buf

    def _handle_message(self, msg):
        log.info("message: %s", msg)

    def _rxloop(self):
        message_cls = None
        state = ReceiveState.idle

        log.debug("starting rx loop, initial state: idle")

        while not self.should_exit:

            if state == ReceiveState.idle:
                sync = self.ser.read(size=1)
                if sync:
                    sync = sync[0]
                    if sync == 0x55:
                        log.debug("state: idle -> header, got 0x55")
                        state = ReceiveState.header
                    else:
                        log.debug("state: idle -> idle, got 0x%x", sync)

            elif state == ReceiveState.header:
                code = self.ser.read(size=1)
                if code:
                    code = code[0]
                    if code in protocol.receive_messages:
                        message_cls = protocol.receive_messages[code]
                        log.debug(
                            "state: header -> payload, got code 0x%x", code
                        )
                        state = ReceiveState.payload
                    else:
                        # log.warning("invalid command code: 0x%x", code)
                        log.debug("state: header -> idle, got code 0x%x", code)
                        state = ReceiveState.idle

            elif state == ReceiveState.payload:
                buf = self._safe_read(message_cls.payload_length + 1)
                log.debug("payload: %s", buf.hex())
                otw_checksum = buf[-1]
                body = buf[:-1]
                checksum = message_cls.compute_checksum(body)
                if False and checksum != otw_checksum:
                    log.warning(
                        "invalid checksum: wanted 0x%x, got 0x%x",
                        otw_checksum,
                        checksum,
                    )
                else:
                    msg = message_cls.parse(body)
                    self._handle_message(msg)
                state = ReceiveState.idle
