# WitMotion Interface

The `witmotion` package is a Python API to interface with WitMotion IMU sensors, such as the [HWT905](https://www.amazon.com/gp/product/B07W3RQJ1M/).

## Quick Start

Install and connect to a device:

    $ pip install witmotion
    $ witmotion-debug --path /dev/ttyUSB0

Get data via a callback:

    from witmotion import IMU

    def callback(msg):
        print(msg)

    imu = IMU()
    imu.subscribe(callback)

Get data via polling interface:

    imu.get_quaternion()
