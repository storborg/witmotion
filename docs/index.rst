WitMotion IMU Python Interface
==============================

This library provides a Python interface to the WitMotion line of
serial-interface IMU sensors. It is tested against the HWT905_, but is also
expected to work for other products, including the HWT901B_.

.. _HWT905: https://www.wit-motion.com/digital-inclinometer/witmotion-hwt905-rs232.html
.. _HWT901B: https://www.wit-motion.com/digital-inclinometer/witmotion-hwt901b-rs232.html

Both polling and callback-oriented APIs are available. At this time, the
``witmotion`` package is only designed to work in a threaded environment, and
does not offer ``async`` APIs.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   quickstart
   debug
   api
   support



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
