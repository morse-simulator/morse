Create your first simulation using the builder API
==================================================

This is an extension of the current :doc:`tutorial <../tutorial>` using the
Morse-builder tool to build your simulation (instead of the Blender graphical
user-interface) and ROS as our middleware (instead of raw Socket). But the
result will be the same!

cf. examples/morse/scenarii/ros_example.py

Create the script
-----------------

In order to use the API, you should import

.. code-block:: python

    from morse.builder.morsebuilder import *


.. note:: The names inside the builder functions must match exactly with
    the names of the .blend files that contain the components.


Add a robot to the scene
++++++++++++++++++++++++
.. code-block:: python

    atrv = Robot('atrv')

Append an actuator
++++++++++++++++++
.. code-block:: python

    motion = Actuator('v_omega')
    motion.translate(z=0.3)
    atrv.append(motion)

Append a Gyroscope sensor
+++++++++++++++++++++++++
.. code-block:: python

    gyroscope = Sensor('gyroscope')
    gyroscope.translate(z=0.83)
    atrv.append(gyroscope)

Adding a middleware
-------------------

Configuring the middlewares
+++++++++++++++++++++++++++
.. code-block:: python

    gyroscope.configure('ros')
    motion.configure('ros')

The middleware components will automatically be appended to the scene when necessary.

Running the simulation
----------------------

Run the simulation
++++++++++++++++++

#. Launch Morse passing your script in argument: ``morse exec mytutorial.py``
#. On a separate terminal, launch the master ROS node using: ``roscore``
#. Press :kbd:`p` to start the Game Engine

Connect with the client
+++++++++++++++++++++++

Use the example client program to test the bindings in the simulation

#. On a separate terminal, navigate to the directory ``$MORSE_ROOT/share/examples/morse/clients/atrv/``
#. Execute the command::

    $ ./ros_v_omega_client.sh

#. Press :kbd:`a` to give speed commands to the robot
#. Type linear (for instance 0.2 m/s) and angular speeds (for instance 0.1
   rad/s), followed by :kbd:`enter` after each
#. The robot should start moving in MORSE
#. Press :kbd:`b` to print the readings of the gyroscope exported by MORSE
#. Press :kbd:`q` to exit the client

Finally exit the simulation, by pressing :kbd:`esc` on the Blender window,
then close Blender by pressing :kbd:`Ctrl-q`, then :kbd:`enter`.

Go further
----------

If you want to learn more about the MORSE-builder API, see the
:doc:`builder documentation <../../../../dev/builder>`.
