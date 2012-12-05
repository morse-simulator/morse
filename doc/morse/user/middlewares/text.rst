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

After the header there is a blank line, and then the data captured by the sensor
is printed with this format::

    ==> Data at X,Y,Z: [3.001482 -5.997612 0.036998] yaw,pitch,roll: [0.002257 0.004322 -0.005083] | time 3.08
        dx = -0.000040
        dy = 0.000089
        dz = 0.000492
        dyaw = -0.000292
        dpitch = -0.000182
        droll = 0.002484

- The first line starts with the string '==>', and shows the position and orientation of the robot at the moment of capturing the data. It also indicates the time, measured in seconds, since the simulation was started.

- The following lines show the contents of the ``local_data`` dictionary of the sensor, each line containing one (key, value) pair. In the example, the data from the :doc:`Odometry sensor <../sensors/odometry>`.

Files
-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/text_datastream.py``

Available methods
-----------------

- ``write_data``: It will print to the file the current position of the robot,
  at the time of the sensor reading, followed by the contents (variables and
  data) in the ``local_data`` dictionary of the associated component.

