YARP-based simulation with cameras
==================================

In this tutorial you can experiment with the cameras in MORSE
and extract the images they capture.
You'll also be able to control the movement of the robot, by giving
it direct coordinates of its destination.

Setup
-----

You need to install YARP and its Python bindings, by following the YARP section in the :doc:`installation notes <../installation>`.

Before running a simulation using YARP, it is necessary to open a new shell terminal and start the ``yarpserver3`` program::

  $ yarpserver3

Configuring the scenario
------------------------

We'll use the Builder API to create the scenario and configure the robot.
You can find the finished file in ``$MORSE_ROOT/share/morse/examples/tutorials/tutorial-2-yarp.py``.

The file itself will be very similar to the one created in the :doc:`first tutorial <tutorial>`.
We will only add the camera, and change the middleware bindings to use YARP.

If you edit the existing file for the previous tutorial, after configuring the robot,
you can add the camera like this:

.. code-block:: python

  Camera = Sensor('video_camera')
  Camera.translate(x=0.2, z=0.9)
  ATRV.append(Camera)

That will include a video camera on top of the robot.

We will also change the motion controller, to have a look at a different
type of controller. We'll use the :doc:`waypoint <../actuators/waypoint>` controller,
which takes as input the coordinates of the destination for the robot.

.. code-block:: python

    Motion_Controller = Actuator('waypoint')
    ATRV.append(Motion_Controller)


Configuring the middlewares
+++++++++++++++++++++++++++

Next we need to tell MORSE how the components will communicate using middlewares.
This is done with these lines:

.. code-block:: python

  Motion_Controller.configure_mw('yarp')
  Pose.configure_mw('yarp')
  Camera.configure_mw('yarp')

This will fill up the necessary information into the ``component_config.py`` file
inside the newly created Blender scenario.


Reading/writing data
--------------------

You can now launch the simulation by pressing :kbd:`p` on the 3Dview of Blender.

When the simulation starts, it will print the names of the YARP ports that have
been created for every corresponding component. These port names can be used to
connect to the component from an external program or client.

Reading from sensors
++++++++++++++++++++

The simplest method to test the reading and writing of data is by using the
terminal clients. For example, to read the Pose data of the robot through a port
named ``/ors/robots/ATRV/Pose/out``, you can type the following in a
terminal::

  $ yarp read /data/in /ors/robots/ATRV/Pose/out

Writing to actuators
++++++++++++++++++++

To enter the destination coordinates for the robot, write them through a port named ``/ors/robots/ATRV/Motion_Controller/in``, using the command::

  $ yarp write /data/out /ors/robots/ATRV/Motion_Controller/in

Then type the three destination coordinates, a toleration distance and the movement speed, separated by spaces, and press :kbd:`enter`. For example::

  5 7 0 0.5 2

Will make the robot move to within 0.5 meters of the coordinates (5.0, 7.0, 0.0), at a speed of 2 m/s.

Displaying an image from the camera
+++++++++++++++++++++++++++++++++++

YARP provides a tool to display the images it receives through a port. This is very
convenient to quickly test that you can transmit video from the simulated environment.
You first need to launch the viewer application::

  $ yarpview /img/read &

This creates a new port called ``/img/read``. Next you need to connect that port with
the output port of MORSE dedicated to the camera, which is: ``/ors/robots/ATRV/CameraMain/out``.
The connection is done with this command::

  $ yarp connect /ors/robots/ATRV/CameraMain/out /img/read

At this moment, you should be able to see the output of the camera mounted on top of the robot.
Instruct the robot to move, using the motion controller, and you'll see the video image being updated.
