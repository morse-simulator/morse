Pan-tilt unit
=============

The purpose of this component is to link together one or more cameras, and
provide them with the possibility to move together as a single unit.  It will
also provide the connection interface to use the information of the cameras
attached to it. In the case of two cameras, it will provide the stereo
information generated from the two camera images.

.. note:: The movement of the pan-tilt unit is implemented with the actuator :doc:`pantilt <../actuators/platine>`

Files
-----
- Blender: ``$MORSE_ROOT/data/morse/components/sensors/morse_ptu.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/ptu.py``

Applicable modifiers
--------------------

The modifiers to the images should be done by configuring the individual
cameras attached to this component.

Services
--------

- **capture**: (asynchronous service) The service takes an integer an argument
  and dispatch the call to all its individual cameras. The service ends when
  each camera has terminated its work.

Related components
------------------

A stereo bench is composed of two regular :doc:`cameras <camera>` parented to a
pan-tilt unit.
