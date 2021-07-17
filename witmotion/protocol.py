import struct
from datetime import datetime, timezone

G = 9.8


class ReceiveMessage:
    payload_length = 8

    @classmethod
    def compute_checksum(cls, body):
        assert len(body) == cls.payload_length
        checksum = 0x55 + cls.code
        for b in body:
            checksum += b
        checksum &= 0xFF
        return checksum


class TimeMessage(ReceiveMessage):
    code = 0x50

    def __init__(self, timestamp):
        self.timestamp = timestamp

    def __str__(self):
        return "time message - timestamp:%s" % self.timestamp

    @classmethod
    def parse(cls, body):
        (year2, month, day, hour, minute, second, millisecond) = struct.unpack(
            ">BBBBBBH", body
        )
        year4 = year2 + 2000
        d = datetime(
            year=year4,
            month=month,
            day=day,
            hour=hour,
            minute=minute,
            second=second,
            microsecond=millisecond * 1000,
        )
        d = d.replace(tzinfo=timezone.utc)
        return cls(timestamp=d.timestamp())


class AccelerationMessage(ReceiveMessage):
    code = 0x51

    def __init__(self, ax, ay, az, temp_celsius):
        self.ax = az
        self.ay = ay
        self.az = az
        self.temp_celsius = temp_celsius

    def __str__(self):
        return "acceleration message - vec:%s,%s,%s temp_celsius:%s" % (
            self.ax,
            self.ay,
            self.az,
            self.temp_celsius,
        )

    @classmethod
    def parse(cls, body):
        (axr, ayr, azr, tempr) = struct.unpack(">HHHH", body)
        ax = (axr / 32768) * 16 * G
        ay = (ayr / 32768) * 16 * G
        az = (azr / 32768) * 16 * G
        temp_celsius = tempr / 100
        return cls(
            ax=ax,
            ay=ay,
            az=az,
            temp_celsius=temp_celsius,
        )


class AngularVelocityMessage(ReceiveMessage):
    code = 0x52

    def __init__(self, wx, wy, wz, temp_celsius):
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.temp_celsius = temp_celsius

    def __str__(self):
        return "angular velocity message - w:%s temp_celsius:%s" % (
            (self.wx, self.wy, self.wz),
            self.temp_celsius,
        )

    @classmethod
    def parse(cls, body):
        (wxr, wyr, wzr, tempr) = struct.unpack(">HHHH", body)
        wx = (wxr / 32768) * 2000
        wy = (wyr / 32768) * 2000
        wz = (wzr / 32768) * 2000
        temp_celsius = tempr / 100
        return cls(
            wx=wx,
            wy=wy,
            wz=wz,
            temp_celsius=temp_celsius,
        )


class AngleMessage(ReceiveMessage):
    code = 0x53

    def __init__(self, roll, pitch, yaw, version):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.version = version

    def __str__(self):
        return "angle message - roll:%d pitch:%s yaw:%s version:%s" % (
            self.roll,
            self.pitch,
            self.yaw,
            self.version,
        )

    @classmethod
    def parse(cls, body):
        (rollr, pitchr, yawr, version) = struct.unpack(">HHHH", body)
        roll = (rollr / 32768) * 180
        pitch = (pitchr / 32768) * 180
        yaw = (yawr / 32768) * 180
        return cls(
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            version=version,
        )


class MagneticMessage(ReceiveMessage):
    code = 0x54

    def __init__(self, x, y, z, temp_celsius):
        self.x = x
        self.y = y
        self.z = z
        self.temp_celsius = temp_celsius

    def __str__(self):
        return "magnetic message - x:%s y:%s z:%s temp_celsius:%s" % (
            self.x,
            self.y,
            self.z,
            self.temp_celsius,
        )

    @classmethod
    def parse(cls, body):
        x, y, z, tempr = struct.unpack(">HHHH", body)
        temp_celsius = tempr / 100
        return cls(
            x=x,
            y=y,
            z=z,
            temp_celsius=temp_celsius,
        )


class QuaternionMessage(ReceiveMessage):
    code = 0x59

    def __init__(self, q):
        self.q = q

    def __str__(self):
        return "quaternion message - q:%s %s %s %s" % self.q

    @classmethod
    def parse(cls, body):
        qr = struct.unpack(">HHHH", body)
        q = tuple(el / 32768 for el in qr)
        return cls(q=q)


receive_messages = {
    cls.code: cls
    for cls in (
        TimeMessage,
        AccelerationMessage,
        AngularVelocityMessage,
        AngleMessage,
        MagneticMessage,
        QuaternionMessage,
    )
}


class ConfigCommand:
    pass
