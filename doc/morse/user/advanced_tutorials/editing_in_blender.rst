Modifying scenes from Blender
=============================

Although now the main method to create scenario files in **morse** is using the
:doc:`Builder API <../../../../user/builder>`, it remains possible and useful to
be able to modify the settings and environment from the Blender interface.

This tutorial explains how to link components into an existing scene.
All the methods here require the use of Blender's user interface and :doc:`short-cuts <../blender_tutorials/basic_blender>`.

We will go through the steps required to manually build "from scratch"
a new robot for simulation. Note that once created, you can save your simulation
scenario as a regular Blender file to replay it directly any time later.

This tutorial assumes MORSE is properly installed. If not, follow the
instructions :doc:`here <../installation>`.

Create the simulation scene
-----------------------------

Load sample file
++++++++++++++++

Open the MORSE simulator with the test file provided with the installation, by using this command::

  $ morse $MORSE_ROOT/share/morse/examples/tutorials/tutorial-1.blend

This will load a scene with a robot in a room with some furniture.

The file::

  $ morse $MORSE_ROOT/share/morse/examples/tutorials/tutorial-1-solved.blend

contains the final scene, as obtained at the end of the tutorial.

Link an actuator
++++++++++++++++

We'll add a :doc:`motion controller <../actuators/v_omega>` to the robot, so that it can receive commands from an external program. The robot will then move according to the instructions received. In this case we'll add a controller that uses linear and angular speed (V, W).

#. With the mouse over the 3D view in Blender, press :kbd:`Ctrl-Alt-O` to open the Load Library browser,
#. Navigate to the directory ``$MORSE_ROOT/data/actuators``,
#. Press :kbd:`Left Mouse Click` over the file ``v_omega.blend``,
#. Press :kbd:`Left Mouse Click` over the item ``Object``,
#. Press :kbd:`Right Mouse Click` over the item ``Motion_Controller``,
#. Press the button **Link/Append from Library**. You'll return to the 3D View.
#. The newly inserted object should be already selected (else select it, either
   by :kbd:`Right Mouse Click` clicking over the object in the 3D View, or
   :kbd:`Left Mouse Click` over the object's name in the Outliner window). The
   object will be highlighted in cyan color, and can not be moved around.
#. Convert the object to local, by pressing :kbd:`l` then hitting :kbd:`enter`. It
   turns to orange outline.
#. With the controller selected, hold down :kbd:`Shift` and then :kbd:`Right Mouse Click` over the robot object,
#. Press :kbd:`Ctrl-p` and then hit :kbd:`enter` make the robot the parent of
   the controller. In the scene outliner, if you press the little ``+`` symbol in
   front of ``ATRV``, you should now see the ``Motion_Controller``.


Link a Pose sensor
++++++++++++++++++

Next we will add a :doc:`pose <../sensors/pose>` sensor to the robot that will report the angles of the robot orientation with respect to the reference axes (yaw, pitch and roll)

#. With the mouse over the 3D view in Blender, press :kbd:`Ctrl-Alt-O` to open the Load Library browser,
#. Navigate to the directory ``$MORSE_ROOT/data/sensors``,
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
        "Pose": [["morse.middleware.socket_mw.MorseSocketClass", "post_message"]],
        "Motion_Controller": [["morse.middleware.socket_mw.MorseSocketClass", "read_message"]],
    }

This specifies that the output of the **Pose** sensor is to be serialized to a socket with the ``MorseSocketClass.post_message`` method and 
the **Motion Controller** reads its input from a socket with ``MorseSocketClass.read_message``.

Running the simulation
----------------------

Run the simulation
++++++++++++++++++

Press :kbd:`p` to start the Game Engine

Connect with the client
+++++++++++++++++++++++

You can connect directly to the simulated sensors/actuators using the ``telnet`` program.
With the configuration provided before, MORSE will create two ports:

* Port 60000 for the **Motion_Controller**
* Port 60001 for the **Pose** sensor

By issuing this command from a terminal you will read a constant feed of the current position
of the robot::

  telnet localhost 60001

To give it movement instructions, you can do the following::

  telnet localhost 60000
  {"v": 2.0, "w": 1.0}

The second line must be given when inside the telnet environment, and will
instruct the robot to move at 2.0 m/s and rotate with an angular speed of
1.0 rad/s.

Of course, ``telnet`` is not the only way to interact with the simulation.
You can connect with the socket ports from another program.

As an example, we have provided a simple Python client program that you
can use to test, and to guide you in creating your own programs.
Just follow these instructions to try the client program:
   
#. On a separate terminal, navigate to the directory ``$MORSE_ROOT/share/morse/examples/clients/atrv/``
#. Execute the command::

    $ python socket_v_omega_client.py

#. Press :kbd:`a` to give speed commands to the robot
#. Type linear (for instance 0.2 m/s) and angular speeds (for instance 0.1 rad/s), followed by :kbd:`enter` after each
#. The robot should start moving in MORSE
#. Press :kbd:`b` to print the readings of the **Pose** sensor exported by MORSE
#. Press :kbd:`q` to exit the client

Finally exit the simulation, by pressing :kbd:`esc` on the Blender window, then close Blender by pressing :kbd:`Ctrl-q`, then :kbd:`enter`.
