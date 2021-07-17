import logging

from enum import Enum
from threading import Thread
from collections import defaultdict

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
        self.subscribers = defaultdict(list)

        # State we have received
        self.last_timestamp = None
        self.last_temp_celsius = None
        self.last_a = None
        self.last_w = None
        self.last_roll = None
        self.last_pitch = None
        self.last_yaw = None
        self.last_mag = None
        self.last_q = None

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

    def subscribe(self, callback, cls=None):
        self.subscribers[cls].append(callback)

    def _handle_message(self, msg):
        log.debug("message: %s", msg)
        for cb in self.subscribers[msg.__class__]:
            cb(msg)
        for cb in self.subscribers[None]:
            cb(msg)
        if isinstance(msg, protocol.TimeMessage):
            self.last_timestamp = msg.timestamp
        elif isinstance(msg, protocol.AccelerationMessage):
            self.last_a = msg.a
            self.last_temp_celsius = msg.temp_celsius
        elif isinstance(msg, protocol.AngularVelocityMessage):
            self.last_w = msg.w
            self.last_temp_celsius = msg.temp_celsius
        elif isinstance(msg, protocol.AngleMessage):
            self.last_roll = msg.roll
            self.last_pitch = msg.pitch
            self.last_yaw = msg.yaw
        elif isinstance(msg, protocol.MagneticMessage):
            self.last_msg = msg.mag
            self.last_temp_celsius = msg.temp_celsius
        elif isinstance(msg, protocol.QuaternionMessage):
            self.last_q = msg.q

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
                if checksum != otw_checksum:
                    log.warning(
                        "invalid checksum: wanted 0x%x, got 0x%x",
                        otw_checksum,
                        checksum,
                    )
                else:
                    msg = message_cls.parse(body)
                    self._handle_message(msg)
                state = ReceiveState.idle

    def get_timestamp(self):
        return self.last_timestamp

    def get_acceleration(self):
        return self.last_a

    def get_angular_velocity(self):
        return self.last_w

    def get_angle(self):
        return self.last_roll, self.last_pitch, self.last_yaw

    def get_magnetic_vector(self):
        return self.last_mag

    def get_quaternion(self):
        return self.last_q

    def save_configuration(self):
        pass

    def send_config_command(self, cmd):
        buf = cmd.serialize()
        log.warning("sending config command %s -> %s", cmd, buf.hex())
        nwritten = self.ser.write(buf)
        log.warning("wrote %d bytes", nwritten)
        if nwritten != len(buf):
            log.warning("possible write failure")

    def set_default_configuration(self):
        pass

    def set_calibration_mode(self, mode):
        if mode == protocol.CalibrationMode.none:
            pass
        elif mode == protocol.CalibrationMode.gyro_accel:
            pass
        elif mode == protocol.CalibrationMode.magnetic:
            pass
        else:
            raise ValueError("invalid calibration mode: %r" % mode)

    def set_installation_direction(self, horizontal):
        self.send_config_command(
            protocol.ConfigCommand(
                register=protocol.Register.direction,
                data=0x00 if horizontal else 0x01,
            )
        )

    def toggle_sleep(self):
        self.send_config_command(
            protocol.ConfigCommand(
                register=protocol.Register.sleep,
                data=0x01,
            )
        )

    def set_algorithm_dof(self, n):
        assert n in (6, 9)
        self.send_config_command(
            protocol.ConfigCommand(
                register=protocol.Register.alg,
                data=(0x00 if n == 9 else 0x01),
            )
        )

    def set_gyro_automatic_calibration(self, enabled=True):
        self.send_config_command(
            protocol.ConfigCommand(
                register=protocol.Register.gyro,
                data=0x00 if enabled else 0x01,
            )
        )

    def set_messages_enabled(self, classes):
        mask = 0
        for cls in classes:
            bitshift = cls.code - 0x50
            mask |= 0x1 << bitshift
        self.send_config_command(
            protocol.ConfigCommand(
                register=protocol.Register.rsw,
                data=mask,
            )
        )

    def set_update_rate(self, rate):
        sel = {
            0.2: protocol.ReturnRateSelect.rate_0_2hz,
            0.5: protocol.ReturnRateSelect.rate_0_5hz,
            1: protocol.ReturnRateSelect.rate_1hz,
            2: protocol.ReturnRateSelect.rate_2hz,
            5: protocol.ReturnRateSelect.rate_5hz,
            10: protocol.ReturnRateSelect.rate_10hz,
            20: protocol.ReturnRateSelect.rate_20hz,
            50: protocol.ReturnRateSelect.rate_50hz,
            100: protocol.ReturnRateSelect.rate_100hz,
            125: protocol.ReturnRateSelect.rate_125hz,
            200: protocol.ReturnRateSelect.rate_200hz,
            "single": protocol.ReturnRateSelect.rate_single,
            None: protocol.ReturnRateSelect.rate_not_output,
        }[rate]
        self.send_config_command(
            protocol.ConfigCommand(
                register=protocol.Register.rate,
                data=sel.value,
            )
        )

    def set_baudrate(self, rate):
        sel = getattr(protocol.BaudRateSelect, "baud_%d" % rate)
        self.send_config_command(
            protocol.ConfigCommand(
                register=protocol.Register.baud,
                data=sel.value,
            )
        )

    def set_acceleration_bias(self, values):
        x, y, z = values

    def set_angular_velocity_bias(self, values):
        x, y, z = values

    def set_magnetic_bias(self, values):
        x, y, z = values
