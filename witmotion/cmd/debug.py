import logging

import time
import sys
import argparse

try:
    import coloredlogs
except ImportError:
    coloredlogs = None

from .. import IMU, protocol

log = logging.getLogger(__name__)


def three_tuple(s):
    parts = s.split(",")
    if len(parts) == 3:
        try:
            return tuple(float(el) for el in parts)
        except ValueError:
            pass
    raise argparse.ArgumentTypeError(
        "'%s' is not a valid 3-tuple. Example: 0.2,1.3,5.6" % s
    )


def main(argv=sys.argv):
    p = argparse.ArgumentParser(description="Emit IMU output")
    p.add_argument("--verbose", action="store_true", help="Verbose debugging")

    # Basic device connectivity
    p.add_argument(
        "--path", default="/dev/ttyUSB0", help="Path to serial device"
    )
    p.add_argument(
        "--baudrate", type=int, default=9600, help="Serial baud rate"
    )

    # To change configuration
    p.add_argument(
        "--reset", action="store_true", help="Reset to default configuration"
    )
    p.add_argument(
        "--set-calibration-mode",
        type=protocol.CalibrationMode,
        choices=tuple(choice.name for choice in protocol.CalibrationMode),
        help="Set calibration mode",
    )
    p.add_argument(
        "--set-algorithm-dof",
        type=int,
        choices=(6, 9),
        help="Set algorithm degrees of freedom",
    )
    p.add_argument(
        "--set-gyro-automatic-calibration",
        type=bool,
        help="Set gyro automatic calibration on or off",
    )
    p.add_argument(
        "--set-installation-direction",
        type=protocol.InstallationDirection,
        choices=tuple(
            choice.name for choice in protocol.InstallationDirection
        ),
        help="Set installation direction",
    )
    p.add_argument("--set-baudrate", type=int, help="Set new baud rate")
    p.add_argument(
        "--set-update-rate", type=float, help="Set new update rate (Hz)"
    )
    p.add_argument(
        "--set-messages",
        help="Comma-separated list of message classes to send",
    )
    p.add_argument(
        "--set-acceleration-bias",
        type=three_tuple,
        help="Set accelaration bias tuple (x,y,z)",
    )
    p.add_argument(
        "--set-angular-velocity-bias",
        type=three_tuple,
        help="Set angular velocity bias tuple (x,y,z)",
    )
    p.add_argument(
        "--set-magnetic-bias",
        type=three_tuple,
        help="Set magnetic bias tuple (x,y,z)",
    )
    p.add_argument(
        "--toggle-sleep", action="store_true", help="Toggle sleep mode"
    )
    p.add_argument(
        "--save", action="store_true", help="Save running configuration to NVM"
    )

    opts = p.parse_args(argv[1:])

    if coloredlogs:
        coloredlogs.install(level="debug" if opts.verbose else "info")
    else:
        logging.basicConfig(
            level=logging.DEBUG if opts.verbose else logging.INFO
        )

    log.debug("opening IMU at %s baudrate %s", opts.path, opts.baudrate)

    imu = IMU(path=opts.path, baudrate=opts.baudrate)

    if opts.reset:
        imu.set_default_configuration()

    if opts.set_calibration_mode:
        imu.set_calibration_mode(opts.set_calibration_mode.value)

    if opts.set_algorithm_dof:
        imu.set_algorithm_dof(opts.set_algorithm_dof)

    if opts.set_gyro_automatic_calibration:
        imu.set_gyro_automatic_calibration(opts.set_gyro_automatic_calibration)

    if opts.set_installation_direction:
        imu.set_installation_direction(opts.set_installation_direction)

    if opts.set_baudrate:
        imu.set_baudrate(opts.set_baudrate)

    if opts.set_update_rate:
        imu.set_update_rate(opts.set_update_rate)

    if opts.set_messages:
        classnames = opts.set_messages.split(",")
        classes = set(getattr(protocol, cn.strip()) for cn in classnames)
        imu.set_messages_enabled(classes=classes)

    if opts.set_acceleration_bias:
        imu.set_acceleration_bias(*opts.set_acceleration_bias)

    if opts.set_angular_velocity_bias:
        imu.set_angular_velocity_bias(*opts.set_angular_velocity_bias)

    if opts.set_magnetic_bias:
        imu.set_magnetic_bias(*opts.set_magnetic_bias)

    if opts.toggle_sleep:
        imu.toggle_sleep()

    if opts.save:
        imu.save_configuration()

    def callback(msg):
        log.info(msg)

    imu.subscribe(callback)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        log.info("exiting")
    finally:
        imu.close()
