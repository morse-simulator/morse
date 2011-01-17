Text middleware
===============

This is the simplest method to export information from MORSE. The data gathered
by components configured to use this "middleware" will be stored in simple text
files. Currently this middleware is only used to output information, so it is
only available for sensors.

File names used in MORSE have the following format:
``[robot_name]_[component_name].txt``. Each file will have a header stating the
name of the robot and the sensor, as well as the position of the sensor with
respect to the center of the robot. The position is given with the format::

  (distance, globalVector(3), localVector(3))

Where ``globalVector`` is a list of 3 elements with the X, Y and Z coordinates
of the sensor with respect to the global origin. ``localVector`` gives the
coordinates with respect to the position of the robot.

Files
-----

- Blender: ``$ORS_ROOT/data/morse/components/middleware/text_empty.blend``
- Python: ``$ORS_ROOT/src/morse/modifiers/text_mw.py``

Available methods
-----------------

- ``write_data``: It will print to the file the current position of the robot,
  at the time of the sensor reading, followed by the contents (variables and
  data) in the ``local_data`` dictionary of the associated component.

