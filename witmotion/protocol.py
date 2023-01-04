import struct
from datetime import datetime, timezone
from enum import Enum
import math

G = 9.8


class ReceiveMessage(object):
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
            "<BBBBBBH", body
        )
        year4 = year2 + 1970
        d = datetime(
            year=year4,
            month=month + 1,
            day=day + 1,
            hour=hour,
            minute=minute,
            second=second,
            microsecond=millisecond * 1000,
        )
        d = d.replace(tzinfo=timezone.utc)
        return cls(timestamp=d.timestamp())


class AccelerationMessage(ReceiveMessage):
    code = 0x51

    def __init__(self, a, temp_celsius):
        self.a = a
        self.temp_celsius = temp_celsius

    def __str__(self):
        return "acceleration message - vec:%s temp_celsius:%s" % (
            self.a,
            self.temp_celsius,
        )

    @classmethod
    def parse(cls, body):
        (axr, ayr, azr, tempr) = struct.unpack("<hhhh", body)
        a = (
            (axr / 32768) * 16 * G,
            (ayr / 32768) * 16 * G,
            (azr / 32768) * 16 * G,
        )
        temp_celsius = tempr / 100
        return cls(
            a=a,
            temp_celsius=temp_celsius,
        )


class AngularVelocityMessage(ReceiveMessage):
    code = 0x52

    def __init__(self, w, temp_celsius):
        self.w = w
        self.temp_celsius = temp_celsius

    def __str__(self):
        return "angular velocity message - w:%s temp_celsius:%s" % (
            self.w,
            self.temp_celsius,
        )

    @classmethod
    def parse(cls, body):
        (wxr, wyr, wzr, tempr) = struct.unpack("<hhhh", body)
        w = (
            (wxr / 32768) * 2000,
            (wyr / 32768) * 2000,
            (wzr / 32768) * 2000,
        )
        temp_celsius = tempr / 100
        return cls(
            w=w,
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
        return (
            "angle message - roll:%0.1f pitch:%0.1f yaw:%0.1f version:%s"
            % (
                self.roll,
                self.pitch,
                self.yaw,
                self.version,
            )
        )

    @classmethod
    def parse(cls, body):
        (rollr, pitchr, yawr, version) = struct.unpack("<hhhh", body)
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

    def __init__(self, mag, temp_celsius):
        self.mag = mag
        self.temp_celsius = temp_celsius

    def __str__(self):
        return "magnetic message - vec:%s temp_celsius:%s" % (
            self.mag,
            self.temp_celsius,
        )

    @classmethod
    def parse(cls, body):
        x, y, z, tempr = struct.unpack("<hhhh", body)
        mag = (x, y, z)
        temp_celsius = tempr / 100
        return cls(
            mag=mag,
            temp_celsius=temp_celsius,
        )


class PressureMessage(ReceiveMessage):
    code = 0x56

    def __init__(self, pressure_pa, height_cm):
        self.pressure_pa = pressure_pa
        self.height_m = height_cm / 100.0

    def __str__(self):
        return "pressure message - pressure_pa:%d pressure_alt_m:%.2f" % (
            self.pressure_pa,
            self.height_m,
        )

    @classmethod
    def parse(cls, body):
        pressure_pa, height_cm = struct.unpack("<ii", body)
        return cls(
            pressure_pa=pressure_pa,
            height_cm=height_cm
        )


class LatLonMessage(ReceiveMessage):
    code = 0x57

    def __init__(self, lat_d, lat_m, lon_d, lon_m):
        self.lat_d = lat_d
        self.lat_m = lat_m
        self.lon_d = lon_d
        self.lon_m = lon_m
        self.lat = float(lat_d) + lat_m/60.0
        self.lon = float(lon_d) + lon_m/60.0

    def __str__(self):
        return "lat/lon message - lat:%.5f lon:%.5f" % (
            self.lat,
            self.lon,
        )

    @classmethod
    def parse(cls, body):
        lat_i, lon_i = struct.unpack("<ii", body)

        F = 1e7

        lat_d = int(math.trunc(lat_i/F))
        lat_m = (lat_i-lat_d*F)/100000

        lon_d = int(math.trunc(lon_i/F))
        lon_m = (lon_i-lon_d*F)/100000

        return cls(
            lat_d=lat_d,
            lat_m=lat_m,
            lon_d=lon_d,
            lon_m=lon_m,
        )

class GroundSpeedMessage(ReceiveMessage):
    code = 0x58

    def __init__(self, height_m, yaw_d, s_kmh):
        self.height_m = height_m
        self.yaw_d = yaw_d
        self.speed_kmh = s_kmh

    def __str__(self):
        return "ground speed message - height:%s yaw:%s speed:%s" % (
            self.height_m,
            self.yaw_d,
            self.speed_kmh,
        )

    @classmethod
    def parse(cls, body):
        height, yaw, s = struct.unpack("<hhi", body)
        return cls(
            height_m=height/10.0,
            yaw_d=yaw/10.0,
            s_kmh=s/1000.0,
        )


class QuaternionMessage(ReceiveMessage):
    code = 0x59

    def __init__(self, q):
        self.q = q

    def __str__(self):
        return "quaternion message - q:%s %s %s %s" % self.q

    @classmethod
    def parse(cls, body):
        qr = struct.unpack("<hhhh", body)
        q = tuple(el / 32768 for el in qr)
        return cls(q=q)


class GNSSAccuracyMessage(ReceiveMessage):
    code = 0x5A

    def __init__(self, n, loc, horiz, vert):
        self.n = n
        self.loc = loc
        self.horiz = horiz
        self.vert = vert

    def __str__(self):
        return "gnss accuracy message - n:%d loc:%s horiz:%s vert:%s" % (
            self.n,
            self.loc,
            self.horiz,
            self.vert,
        )

    @classmethod
    def parse(cls, body):
        n, l, h, v = struct.unpack("<HHHH", body)
        return cls(
            n=n,
            loc=l/32768.0,
            horiz=h/32768.0,
            vert=v/32768.0,
        )


receive_messages = {
    cls.code: cls
    for cls in (
        TimeMessage,
        AccelerationMessage,
        AngularVelocityMessage,
        AngleMessage,
        MagneticMessage,
        PressureMessage,
        LatLonMessage,
        GroundSpeedMessage,
        QuaternionMessage,
        GNSSAccuracyMessage,
    )
}


class CalibrationMode(Enum):
    """
    Available sensor calibration modes.
    """

    none = 0
    "No calibration mode enabled."

    gyro_accel = 1
    "Enable gyroscope and accelerometer calibration."

    magnetic = 2
    "Enable magnetic calibration."


class InstallationDirection(Enum):
    """
    Available installation directions.
    """

    horizontal = 0x00
    "Device installed horizontally (default)."

    vertical = 0x01
    "Device installed vertically."


class ReturnRateSelect(Enum):
    rate_0_2hz = 0x01
    rate_0_5hz = 0x02
    rate_1hz = 0x03
    rate_2hz = 0x04
    rate_5hz = 0x05
    rate_10hz = 0x06
    rate_20hz = 0x07
    rate_50hz = 0x08
    rate_100hz = 0x09
    rate_125hz = 0x0A
    rate_200hz = 0x0B
    rate_single = 0x0C
    rate_not_output = 0x0D


class BaudRateSelect(Enum):
    baud_4800 = 0x01
    baud_9600 = 0x02
    baud_19200 = 0x03
    baud_38400 = 0x04
    baud_57600 = 0x05
    baud_115200 = 0x06
    baud_230400 = 0x07
    baud_460800 = 0x08
    baud_921600 = 0x09


class Register(Enum):
    save = 0x00
    calsw = 0x01
    rsw = 0x02
    rate = 0x03
    baud = 0x04
    axoffset = 0x05
    ayoffset = 0x06
    azoffset = 0x07
    gxoffset = 0x08
    gyoffset = 0x09
    gzoffset = 0x0A
    hxoffset = 0x0B
    hyoffset = 0x0C
    hzoffset = 0x0D
    sleep = 0x22
    direction = 0x23
    alg = 0x24
    mmyy = 0x30
    hhdd = 0x31
    ssmm = 0x32
    ms = 0x33
    ax = 0x34
    ay = 0x35
    az = 0x36
    gx = 0x37
    gy = 0x38
    gz = 0x39
    hx = 0x3A
    hy = 0x3B
    hz = 0x3C
    roll = 0x3D
    pitch = 0x3E
    yaw = 0x3F
    temp = 0x40
    q0 = 0x51
    q1 = 0x52
    q2 = 0x53
    q3 = 0x54
    gyro = 0x63

    unknown_config_cmd = 0x69


class ConfigCommand:
    def __init__(self, register, data):
        self.register = register
        self.data = data

    def __str__(self):
        return "config command - register %s -> data %s" % (
            self.register.name,
            self.data,
        )

    def serialize(self):
        return struct.pack(
            "<BBBH",
            0xFF,
            0xAA,
            self.register.value,
            self.data,
        )
