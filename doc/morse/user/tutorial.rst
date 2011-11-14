Create your first simulation using the builder API
==================================================

This tutorial will guide you through creating a simple scenario where you can
control a mobile robot and read data from its sensors. It makes use of the
:doc:`Builder API <../../../../dev/builder>` of MORSE. This consists on a series
of Python functions to define these elements of a simulation scene:

 - The robots to use
 - The components attached to them
 - The middleware bindings used for communication
 - The environment of the simulation

Any of the elements in the :doc:`MORSE component library <../components_library>`
can be included in this description.

The end result is a Python script that can be executed from MORSE to generate
a ``.blend`` file. There is no previous Blender knowledge necessary to construct
a simulation scene.

cf. examples/morse/scenarii/tutorial-1-sockets.py

Create the script
-----------------

For this tutorial, you will describe all the elements of your scene (robots,
sensors, actuators) in a Python script, and then execute it using **morse**.
We describe here how to create the script:

Open a text file, and give it an appropriate name. For instance: ``robot_scene.py``.
Inside it, add the code pieces described below.

In order to use the API, you should import some **morse** libraries:

.. code-block:: python

    from morse.builder.morsebuilder import *


Then you will make calls to predefined functions to create and configure the
components necessary in your scene.

.. note:: The names inside the builder functions must match exactly with
    the names of the .blend files that contain the components. Check the
    component library for references.


Add a robot to the scene
++++++++++++++++++++++++
The robot is the base where we will install other sensors and actuators.

.. code-block:: python

    atrv = Robot('atrv')

Append an actuator
++++++++++++++++++
We'll add a **v, omega** actuator. This one controls the robot by changing the linear and
angular velocity of the movement.

.. code-block:: python

    motion = Actuator('v_omega')
    motion.translate(z=0.3)
    atrv.append(motion)

Append a Pose sensor
+++++++++++++++++++++++++
We'll add the **Pose** sensor, which provides us with the location and rotation of the robot.
The data it sends back is the *(x, y, z)* coordinates, and the *(yaw, pitch, roll)* orientation.

.. code-block:: python

    pose = Sensor('pose')
    pose.translate(z=0.83)
    atrv.append(pose)

Adding a middleware
-------------------

The simplest way to test MORSE out-of-the box is to use **sockets** to access the
**services** provided by the components. This method has no software requirements other
than the base MORSE installation.

Configuring the middlewares
+++++++++++++++++++++++++++

.. code-block:: python

    pose.configure_service('socket')
    motion.configure_service('socket')



Finalising the scene
--------------------

Every builder script must finish with an environment description. This is mandatory, or
else the scene will not be created. The parameter for the **Environment** method is the
name of a .blend file that should be located in ``$MORSE_ROOT/share/data/morse/environments/``.

The Environment object also provides additional options to place and aim the default camera,
by using the methods ``aim_camera`` and ``place_camera``.

.. code-block:: python

    env = Environment('indoors-1/indoor-1')
    env.aim_camera([1.0470, 0, 0.7854])


Now save your script file!!!


Running the simulation
----------------------

Run the simulation
++++++++++++++++++

#. Launch Morse passing your script in argument::

    $ morse exec robot_scene.py

#. Place your mouse inside the 3D view of the scenario
#. Press :kbd:`p` to start the Game Engine

Connect with the client
+++++++++++++++++++++++

Using sockets to connect to robot services is the simplest way to interact
with the simulation. You can talk with **morse** through a simple telnet connection.
On a separate terminal, type::

  $ telnet localhost 4000

Port 4000 is the default used by **morse**.
We can try out the simulation by giving instructions in the telnet terminal.

You will need to use the services provided by the components we installed in the robot.
To make the robot move in a circle, with linear speed 2 m/s and angular speed -1 rad/s, use this instruction::

  id1 Motion_Controller set_speed [2, -1]

To ask the **Pose** sensor for the data it contains, use this command::

  id2 Pose get_local_data []

The format of these commands is simple, they are composed of four parts:

#. The **id** of the request. It is a string to identify the individual instructions
#. The name of the component. This is the name of the Blender object in the scene that
    represents the sensor or actuator
#. The name of the service. These vary for each component, and are listed in the :doc:`component library <../components_library>` section
#. The list of parameters for the function. Must be enclosed in brackets and separated by commas

Try giving the **Motion_Controller** different speeds, and asking the **Pose** at different locations.
Finally exit the simulation, by pressing :kbd:`esc` on the Blender window.
You can save your scene as a Blender file, and then run it directly using **morse**.
To close Blender, press :kbd:`Ctrl-q`, then :kbd:`enter`.

Go further
----------

If you want to learn more about the MORSE-builder API, see the
:doc:`builder documentation <../../../../dev/builder>`.
