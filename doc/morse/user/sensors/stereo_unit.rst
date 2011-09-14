Stereo Camera unit
==================

The purpose of this component is to link together one or more cameras, and
provide them with the possibility to move together as a single unit.  It will
also provide the connection interface to use the information of the cameras
attached to it. In the case of two cameras, it will provide the stereo
information generated from the two camera images.

Files
-----
- Blender: ``$MORSE_ROOT/data/morse/sensors/stereo_unit.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/stereo_unit.py``

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

A stereo unit needs to be the parent of one or more :doc:`cameras <video_camera>`.
Otherwise, it does no useful function.

The movement of the stereo unit is implemented by making it the child of a
:doc:`Pan-Tilt unit <../actuators/ptu>` actuator.
