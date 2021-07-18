Quick Start
===========

Install and connect to a device::

    $ pip install witmotion
    $ witmotion-debug --path /dev/ttyUSB0

Get data via a callback:

.. code-block:: python

    from witmotion import IMU

    def callback(msg):
        print(msg)

    imu = IMU()
    imu.subscribe(callback)

Get data via polling interface:

.. code-block:: python

    imu.get_quaternion()
