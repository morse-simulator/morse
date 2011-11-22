PTU posture sensor
==================

Simple sensor that provides the current rotation angles of the *pan* and *tilt*
segments of the :doc:`PTU actuator <../actuators/ptu>`.
The angles returned are in radians in the range (-pi, pi).

To use this sensor, it must be installed as the parent of the PTU actuator.
For example, using the Builder API, the PTU actuator and PTU posture
sensor must be defined as ::

    PTU_posture = Sensor('ptu_posture')
    PTU_posture.translate(x=0.2020, y=0.0072, z=1.4400)
    Jido.append(PTU_posture)

    PTU = Actuator('ptu')
    PTU_posture.append(PTU)


Files
-----

- Blender: ``$MORSE_ROOT/data/morse/sensors/ptu_posture.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/ptu_posture.py``

Local data
----------

- **pan**: (float) X coordinate of the sensor
- **tilt**: (float) Y coordinate of the sensor

.. note:: The angles are given with respect to the orientation of the robot

Applicable modifiers
--------------------

No applicable modifiers at the moment.

Related components
------------------

This sensor will only be useful when paired with a :doc:`PTU actuator <../actuators/ptu>`.
It reads the data directly from the actuator that is set as its child.
It will generate errors if not set in this way.
