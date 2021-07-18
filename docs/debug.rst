Command-line Debug Tool
=======================

The package includes a command-line debug and configuration tool. This tool can
be used to update all parameters of the running or saved configuration on a
WitMotion IMU device, or just display data to verify functionality.

Typical usage:

.. code-block::

    witmotion-debug --path /dev/ttyUSB0

Additional argumnets:

.. code-block::

    optional arguments:
      -h, --help            show this help message and exit
      --verbose             Verbose debugging
      --path PATH           Path to serial device
      --baudrate BAUDRATE   Serial baud rate
      --reset               Reset to default configuration
      --set-calibration-mode {none,gyro_accel,magnetic}
                            Set calibration mode
      --set-algorithm-dof {6,9}
                            Set algorithm degrees of freedom
      --set-gyro-automatic-calibration SET_GYRO_AUTOMATIC_CALIBRATION
                            Set gyro automatic calibration on or off
      --set-installation-direction {horizontal,vertical}
                            Set installation direction
      --set-baudrate SET_BAUDRATE
                            Set new baud rate
      --set-update-rate SET_UPDATE_RATE
                            Set new update rate (Hz)
      --set-messages SET_MESSAGES
                            Comma-separated list of message classes to send
      --set-acceleration-bias SET_ACCELERATION_BIAS
                            Set accelaration bias tuple (x,y,z)
      --set-angular-velocity-bias SET_ANGULAR_VELOCITY_BIAS
                            Set angular velocity bias tuple (x,y,z)
      --set-magnetic-bias SET_MAGNETIC_BIAS
                            Set magnetic bias tuple (x,y,z)
      --toggle-sleep        Toggle sleep mode
      --save                Save running configuration to NVM
