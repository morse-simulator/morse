Modifying scenes from Blender
=============================

Although now the main method to create scenario files in **morse** is using the
:doc:`Builder API <../../../../dev/builder>`, it remains possible and useful to
be able to modify the settings and environment from the Blender interface.

This tutorial explains how to link components into an existing scene.
All the methods here require the use of Blender's user interface and :doc:`short-cuts <advanced_blender>`.

We will go through the steps required to manually build "from scratch"
a new robot for simulation. Note that once created, you can save your simulation
scenario as a regular Blender file to replay it directly any time later.

This tutorial assumes MORSE is properly installed. If not, follow the
instructions :doc:`here <installation>`.

Create the simulation scene
-----------------------------

Load sample file
++++++++++++++++

Open the MORSE simulator with the test file provided with the installation, by using this command::

  $ morse $MORSE_ROOT/share/examples/morse/tutorials/tutorial-1.blend

This will load a scene with a robot in a room with some furniture.

The file::

  $ morse $MORSE_ROOT/share/examples/morse/tutorials/tutorial-1-solved.blend

contains the final scene, as obtained at the end of the tutorial.

Link an actuator
++++++++++++++++

We'll add a motion controller to the robot, so that it can receive commands from an external program. The robot will then move according to the instructions received. In this case we'll add a controller that uses linear and angular speed (V, W).

#. With the mouse over the 3D view in Blender, press :kbd:`Ctrl-Alt-O` to open the Load Library browser,
#. Navigate to the directory ``$MORSE_ROOT/data/morse/actuators``,
#. Press :kbd:`Left Mouse Click` over the file ``v_omega.blend``,
#. Press :kbd:`Left Mouse Click` over the item ``Object``,
#. Press :kbd:`Right Mouse Click` over the item ``Motion_Controller``,
#. Press the button **Link/Append from Library**. You'll return to the 3D View.
#. The newly inserted object should be already selected (else select it, either
   by :kbd:`Right Mouse Click` clicking over the object in the 3D View, or
   :kbd:`Left Mouse Click` over the object's name in the Outliner window). The
   object will be highlighted in cyan color, and can not be moved around.  #.
   Convert the object to local, by pressing :kbd:`l` then hitting :kbd:`enter`. It
   turns to orange outline.
#. With the controller selected, hold down :kbd:`Shift` and then :kbd:`Right Mouse Click` over the robot object,
#. Press :kbd:`Ctrl-p` and then hit :kbd:`enter` make the robot the parent of
   the controller. In the scene outliner, if you press the little ``+`` symbol in
   front of ``ATRV``, you should now see the ``Motion_Controller``.

.. _link-gyroscope-sensor:

Link a Pose sensor
++++++++++++++++++

Next we will add a sensor to the robot that will report the angles of the robot orientation with respect to the reference axes (yaw, pitch and roll)

#. With the mouse over the 3D view in Blender, press :kbd:`Ctrl-Alt-O` to open the Load Library browser,
#. Navigate to the directory ``$MORSE_ROOT/data/morse/sensors``,
#. Press :kbd:`Left Mouse Click` over the file ``pose.blend``,
#. Press :kbd:`Left Mouse Click` over the item ``Object``,
#. Press select all items (``Pose_sensor`` and ``Pose_mesh``), by holding :kbd:`Shift` down, and load them.
#. Convert the two object to local, by pressing :kbd:`l` then hitting :kbd:`enter`,
#. Switch to front view by pressing :kbd:`1` (or use the ``View`` menu at the bottom of the 3D view),
#. Press :kbd:`g`, then move the ``Pose_sensor`` object on the top of the robot (you can constraint the translation on the Z axis by simply pressing :kbd:`Z`),
#. Press :kbd:`Left Mouse Click` to accept the movement,
#. With the ``Pose_sensor`` object selected, hold down :kbd:`Shift` and then :kbd:`Right Mouse Click` over the robot object,
#. Press :kbd:`Ctrl-p` and then hit :kbd:`enter` make the robot the parent of the controller.


Adding middleware communication
-------------------------------

Configuring the middlewares
+++++++++++++++++++++++++++

Binding the components in the scene with the middleware is done in a configuration file within the Blender file.

#. On the **Text Editor** window, select the file ``component_config.py``
#. Add the following items to the ``component_mw`` dictionary::
  
    component_mw = {
        "Gyroscope": [["Socket", "post_message"]],
        "Motion_Controller": [["Socket", "read_message"]],
    }

This specifies that the output of the gyroscope sensor is to be serialized to a socket with the ``MorseSocketClass.post_message`` method and 
the motion controller reads its input from a socket with ``MorseSocketClass.read_message``.

Running the simulation
----------------------

Run the simulation
++++++++++++++++++

Press :kbd:`p` to start the Game Engine

Connect with the client
+++++++++++++++++++++++

Use the example client program to test the bindings in the simulation

#. On a separate terminal, navigate to the directory ``$MORSE_ROOT/share/examples/morse/clients/atrv/``
#. Execute the command::

    $ python socket_v_omega_client.py

#. Press :kbd:`a` to give speed commands to the robot
#. Type linear (for instance 0.2 m/s) and angular speeds (for instance 0.1 rad/s), followed by :kbd:`enter` after each
#. The robot should start moving in MORSE
#. Press :kbd:`b` to print the readings of the gyroscope exported by MORSE
#. Press :kbd:`q` to exit the client

Finally exit the simulation, by pressing :kbd:`esc` on the Blender window, then close Blender by pressing :kbd:`Ctrl-q`, then :kbd:`enter`.
