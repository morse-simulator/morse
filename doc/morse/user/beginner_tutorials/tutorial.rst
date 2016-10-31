Create your first simulation :tag:`builder` :tag:`socket`
=========================================================

This tutorial will guide you through the creation of a simple simulation where
you can control a mobile robot and read data from its sensors.

It makes use of MORSE's :doc:`Builder API <../../../../user/builder>`: a set of
Python functions that allows you to define:

- The robots to use
- The components attached to them
- The middleware bindings used for communication
- The simulation's physical environment

During the tutorial, we will write a Python script that can be executed by MORSE to
create your simulation and start it running.

.. note:: This tutorial's complete script is available here:
    ``$MORSE_ROOT/share/morse/examples/tutorials/tutorial-1-sockets.py``, where
    ``$MORSE_ROOT`` is your installation prefix (typically ``/usr/local/``).

Create the script
-----------------

Create a new script in your favorite Python editor, and give it an appropriate
name. For instance: ``robot_scene.py``.

In order to use the API, you must first import the ``morse.builder`` module:

.. code-block:: python

    from morse.builder import *


Then you will make calls to predefined functions to create and configure the
components required for your scene.

MORSE knows three main **components**: the **robots**, the **sensors** and the
**actuators** (the robots are mostly supports for sensors and actuators).

All these components live in an **environment**, which may be any
physics-enabled 3D model.

The behaviour of these components can be altered by **modifiers** and their
interactions with software running *outside* the simulator rely on
**middlewares**.


Add a robot to the scene
++++++++++++++++++++++++

The robot is the basic component to which we add sensors and actuators.

.. code-block:: python

    atrv = ATRV()

Here, we simply import a standard ATRV 4-wheeled outdoor robot.

The ``ATRV`` is already known to MORSE, as you can see in the :doc:`component library
<../../components_library>`.


Append an actuator
++++++++++++++++++

Now, let's add a :doc:`v, omega <../actuators/v_omega>` actuator. This
is used to change the robot's linear and angular velocity.

.. code-block:: python

    motion = MotionVW()
    motion.translate(z=0.3)
    atrv.append(motion)

The ``append`` method *parents* the actuator to the robot, (i.e., makes
the actuator one of the robot's child components).

Append a sensor
+++++++++++++++

We can now add a :doc:`Pose <../sensors/pose>` sensor, which will report
the robot's location and rotation.

The data it sends back are the *(x, y, z)* coordinates, and the *(yaw, pitch,
roll)* orientation. For any component, you can learn what data it can
export from its documentation, e.g., :doc:`Pose <../sensors/pose>` 

.. code-block:: python

    pose = Pose()
    pose.translate(z=0.83)
    atrv.append(pose)

Configuring the middlewares
---------------------------

The simplest way to test MORSE is to use the basic **socket** to access the
**data-streams** and **services** provided by the components. This method has
no software requirements other than the base MORSE installation.

You need to tell MORSE how each of the components attached to a robot will communicate
with the outside world. This is done as follows:

.. code-block:: python

    pose.add_stream('socket')
    pose.add_service('socket')
    motion.add_service('socket')

Each of the components can use a different middleware, enabling the use of
MORSE in an heterogeneous environment. You can check the :doc:`full list of
supported middlewares <../integration>` for reference.

Finalising the scene
--------------------

Every builder script must finish with the creation of an environment.

The parameter for the **Environment** method is the name of a Blender
``.blend`` file you provide (with its full path) or a :doc:`pre-defined one
<../../environments>`.

The Environment object also provides additional options to place and aim the
default camera, by using the methods ``set_camera_rotation`` and
``set_camera_location``.

.. code-block:: python

    env = Environment('indoors-1/indoor-1')
    env.set_camera_location([5, -5, 6])
    env.set_camera_rotation([1.0470, 0, 0.7854])


Now save your script file.


Running the simulation
----------------------

Starting the simulation
+++++++++++++++++++++++

Simply run::

    $ morse run robot_scene.py

Alternatively, you can choose to open your simulation in Blender, and
start it from there:

#. Launch MORSE in *edit* mode, passing your script as argument::

    $ morse edit robot_scene.py

#. Place your mouse inside the 3D view of the scenario
#. Press :kbd:`p` to start the Game Engine

Control the simulation with services
++++++++++++++++++++++++++++++++++++

Using sockets to connect to robot services is the simplest way to interact
with the simulation. You can talk to MORSE through a simple telnet connection.
In a separate terminal, type::

  $ telnet localhost 4000

Port 4000 is the default port used by MORSE to expose the **services**.

The motion controller we have added to the robot exports one service,
``set_speed``: to make the robot move in a circle, with linear speed 2 m/s and
angular speed -1 rad/s, type this instruction::

  id1 atrv.motion set_speed [2, -1]

.. note::
    the first part of the request, ``id1`` is any identifier you want. It is useful
    when running *asynchronous services* (*i.e.*, non-blocking) to be notified of the
    service's termination.

.. note::
    the internal name of the component (here, ``atrv.motion``) is displayed
    in the MORSE log at the end of the simulation initialisation.

In the same way, you can query the ``atrv.pose`` sensor for the data it contains::

  id2 atrv.pose get_local_data 

The format of these commands is simple, they are composed of four parts:

#. The request identifier (a name you make up, ideally unique for each request)
#. The name of the object to send the request to
#. The name of the request itself (i.e., the method name)
#. The request's parameters (if any) in JSON format

Try giving the motion controller different speeds, and querying the pose sensor
at different locations.

Accessing the sensors' data streams
+++++++++++++++++++++++++++++++++++

The *Pose* sensor constantly exports its data as a stream.

We can use ``telnet`` to monitor its output.

Since many sensors may output their data-stream on the socket interface, each
of them is assigned a port at runtime. You can retrieve this port either by
looking at MORSE console output, or with the **simulation services**
``list_streams`` and ``get_stream_port``::

  id3 simulation list_streams 
  > id3 SUCCESS ["atrv.pose"]
  id4 simulation get_stream_port ["atrv.pose"]
  > id4 SUCCESS 60000

So, in this case we now know that the pose sensor is exporting its datastream
on port 60000.

Open another ``telnet`` session::

  $ telnet localhost 60000

Your screen should be filled pretty quickly with the sensor's output.

Many actuators also accept a datastream as input to control their behaviour.

To exit the simulation, press :kbd:`Esc` in the Blender window.

What's next?
------------


- You can try to add different components to the robot, by experimenting with
  the various objects available in the :doc:`MORSE component library
  <../../components_library>`.  This is the main reference for robots,
  actuators, and sensors that are available *out of the box* in MORSE.

.. note:: The names you pass to the Builder functions link to the names
    of the Blender ``.blend`` files that contain the components' meshes. They
    are provided for each component in the component library.


- You can also discover how you can :doc:`extend MORSE <../../contributing>` by
  adding your own sensors, actuators, and robots.

- If you want to learn more about the MORSE *Builder API*, see the
  :doc:`builder documentation <../../../../user/builder>`.

- Finally, you can go back to the :doc:`list of tutorials <../../tutorials>`.

