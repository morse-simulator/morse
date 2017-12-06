MAVLINK
=======

MAVLink is a very lightweight, header-only message marshalling library for
micro air vehicles. 

It can pack C-structs over serial channels with high effiency and send these
packets to the ground control station. It is extensively tested on the PX4,
PIXHAWK, APM and Parrot AR.Drone platforms and serves as the communication
backbone for the MCU/IMU communication as well as for Linux interprocess and
ground link communication. 

To use this binding with Morse, you need at least pymavlink >= 1.1.62.

.. note::

    As mavlink messages have strong semantics about used frames, the middleware
    part deals with ENU -> aeronautical frames, so mavlink messages have the
    expected semantics. 

Files
-----

- Python: ``$MORSE_ROOT/src/morse/middleware/mavlink_datastream.py``

.. _mavlink_ds_configuration:

Configuration specifics
-----------------------

When configuring a component to export its data through mavlink, you can pass
several options to tune the behaviour of the exporter and importer.

The option ``device`` allows you to configure the link which is used to exchange
data trough Mavlink. Valid configurations are currently:
    - 'udpout:host:port'
    - 'udpin:host:port'
    - 'tcp:host:port'

.. note ::
    
    Unlike many other middlewares, in Mavlink, multiple sensors /
    actuators can use the same device.

.. code-block :: python

    foo.add_stream('mavlink', device = 'udpout:localhost:14550')


