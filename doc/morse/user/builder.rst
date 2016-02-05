Build your simulations
======================

If you did not check it, we recommend you to first go over the
:doc:`Quickstart<../quickstart>` page.

For a practical reference of the Builder API module methods, see the
:doc:`Builder Overview<builder_overview>`.

Creating a new simulation
-------------------------

In MORSE, we call a *simulation environment* a main *simulation scenario* (a
Python script that describe the environment, the static objects, the robots,
the sensors and/or actuators, etc.) and a set of optional resources, that
may comprise models (as Blender ``.blend`` files) and implementation of custom
components.

These files are usually stored in a single self-contained folder (which ease
sharing), but since regular Python's ``import`` mechanism works as expected,
you can also organize your files differently (for instance, to share a
complex robot description between several simulation scenarios).

To get you started quickly, MORSE provides a special command::

 $ morse create <simulation name>

For instance::

 $ morse create my_project

creates a new folder called ``my_project`` in the current directory, and also
add an entry to the MORSE configuration file (usually located in
``$HOME/.morse/config``) that allows you to run your simulation from anywhere,
with a simple ``morse run my_project``.

``my_project`` is your new **simulation environment**. The ``my_project/``
folder contains one important file, ``default.py``. This is a sample
**simulation scenario** (also called **Builder script**). Open it to see how
it works.

``my_project/scripts`` contains a sample *client* script that shows how to
connect to MORSE through Python. You can safely delete it if you do not need
it.

How to build a simulation scenario suitable to your need? The remaining of this
page details MORSE's ``Builder API``. It is a set of components and methods
that let you build a simulation scene.

Builder scripts
---------------

Simulation scenarios (or ``Builder`` scripts) rely on the **Builder API** to
define the components, environments and middlewares that are going to be
used in the simulation. When running MORSE, the simulator interprets the
scripts to build the complete simulation as a Blender scene, configures the
components, and starts the Blender 3D Engine.

Since these scripts are regular Python scripts, you can easily create one
function or class for each of your robots. This way, you can quickly reuse them
in different contexts.

A basic builder script looks like:

.. code-block:: python

    from morse.builder import *
    # [...]
    env = Environment('indoors-1/indoor-1', fastmode=False)

- The first line tells MORSE that this is a builder script.
- The second is a comment, it's where you will add robots, sensors and actuators.
- Then we create an environment. The environment instance, here ``env``, will let you
  tune some simulation parameters. See :py:mod:`morse.builder.environment` for a
  list of methods. If you set the optional parameter ``fastmode`` to ``True``,
  MORSE switches to a simpler rendering method (wireframe) which leads to much
  faster performances, but you can not use vision-based sensors like cameras in
  this mode.

.. note::

    The Environment object must be the last thing created in the builder
    script.

If your edit this script in MORSE, you should see the ``'indoors-1/indoor-1'``
scene:

.. image:: ../../media/builder/morse_builder_1empty.png
   :width: 400
   :align: center
.. MORSE Builder empty


Since there is no robot yet, you won't be able launch the simulation.

The environment comes from the file ``data/environments/indoors-1/indoor-1.blend``,
you could also tell MORSE to load an environment from any location on your system
by adding the ``.blend`` extension. *eg*, ``Environment('../my_house.blend')`` will
load the file ``my_house.blend`` in the parent directory where you run morse from.

You can also set a ``MORSE_RESOURCE_PATH`` environment variable with::

    export MORSE_RESOURCE_PATH="/path/number/one:/path/number/two"

where MORSE will be looking for components. The default place it looks in is
``$MORSE_ROOT/share/morse/data`` (typically ``/usr/local/share/morse/data``)

An additional option is to place and aim the default camera, by using the methods
:py:meth:`morse.builder.environment.Environment.set_camera_rotation` and
:py:meth:`morse.builder.environment.Environment.set_camera_location`.

.. code-block:: python

    env = Environment('land-1/trees')
    env.set_camera_location([-5.0, 5.0, 3.0])
    env.set_camera_rotation([1.0470, 0, -0.7854])

.. note::
    You can also edit a builder script directly in MORSE, by calling ``morse edit my_builder_script.py``.
    This let you build your environment with Blender's GUI. Save it as a regular Blender file, and 
    run it directly: ``morse run my_sim.blend``. Be aware that MORSE does not support converting such a Blender
    simulation back to a Python Builder script.

Adding a robot
++++++++++++++

Let's add a robot to our scene:

.. code-block:: python

    from morse.builder import *

    # Append ATRV robot to the scene
    robot = ATRV()

    env = Environment('indoors-1/indoor-1')

.. image:: ../../media/builder/morse_builder_2robot.png
   :width: 400
   :align: center
.. MORSE Builder robot (ATRV)


You should see the ``ATRV`` at the center of the scene.

.. warning::

    Handling of loop in builder script is a bit complex. There is two possible
    solution to handle properly loop at the builder level. The first one is
    to name explicitly your robot such as:

    .. code-block:: python

        for i in range(1, 5): 
            robot = ATRV('robot')

    The second solution is to use the special method ``close_context`` such
    as:

    .. code-block:: python

        for i in range(1, 5): 
            robot = ATRV()
            AbstractComponent.close_context()




Adding sensors and actuators
++++++++++++++++++++++++++++

A robot needs informations about it's location, and to apply some movements.
There are different way to achieve this, in our example, we will use a ``Pose``
sensor and a ``Motion`` controller 'v-omega'.

.. code-block:: python

    from morse.builder import *

    # Append ATRV robot to the scene
    robot = ATRV()

    # Append an actuator
    motion = MotionVW()
    robot.append(motion)

    # Append a sensor
    pose = Pose()
    pose.translate(z = 0.75)
    robot.append(pose)

    # Configure the robot on the 'socket' interface
    robot.add_default_interface('socket')

    env = Environment('indoors-1/indoor-1')

The last line configure the robot's components on socket, for more information
about services and datastreams, go to `Middleware configuration`_


.. note::
    In this example, the motion controller in your simulation will be named
    ``motion``.

    The name is used by MORSE to refer to the component in the simulator
    interface. Each middleware has it's own naming convention, but for
    instance with the basic ``socket`` interface, you can send a command to
    the motion controller like that::

        $ telnet localhost 4000
        Connected to localhost.
        > req1 motion set_speed [1.0, 0.002]
        req1 OK


Position a component
++++++++++++++++++++

There are 2 transformations you can give to a component: ``translate(x, y, z)``
and ``rotate(x, y, z)``.

* The translation will add (x, y, z) to the current object location
  (default: x=0, y=0, z=0, unit: meter).
* The rotation is an `euler rotation
  <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.rotation_euler>`_
  relative to the object's center (default: x=0, y=0, z=0, unit: radian).

.. code-block:: python

    motion.translate(x=.2, z=1)
    atrv.rotate(z=3.14)



Naming of components
--------------------

You can set the name of a component through the setter ``name``::

    mouse = ATRV()
    mouse.name = "jerry"


If you do not explicitly set the name of your components, MORSE name them
automatically (including the Blender objects representing your components)
based on **the name of the variable used in your Builder script**.

In all cases, the components names are automatically **prefixed with their
parents**, to prevent name collision.

Let take an example. Consider this script, with two robots::

    from morse.builder import *

    tom = ATRV()
    lefteye = VideoCamera()
    ptu = PTU()
    righteye = VideoCamera()
    righteye.name = "blindeye"

    tom.append(lefteye)
    ptu.append(righteye)
    tom.append(ptu)

    mouse = ATRV()
    mouse.name = "jerry"
    cam = VideoCamera()
    mouse.append(cam)

    env = Environment('indoors-1/indoor-1')


If you open it in MORSE for edition (with ``morse edit``) and you look at the
outliner, you see that the hierarchy of objects looks like that:

.. code-block:: none

    tom
     |-> tom.lefteye
     |-> tom.ptu
        |-> tom.ptu.blindeye
    jerry
     |-> jerry.cam

``tom`` comes from the variable name, whereas ``jerry`` was manually set.

.. note::
    Automatic renaming only works for components *visible* from your script
    (*ie*, a component declared in a function or class, which is not assigned to
    a variable that belongs to your ``Builder`` script, will not be renamed) or
    components that were appended to a component which is visible.

.. note::
    The renaming process works only for object created before the Environment
    object. Make sure to create this one at the end of the builder script.

.. note::
    If name collisions occur anyway, Blender automatically adds an incremental
    suffix like ``.001``, ``.002``, etc.

.. note::
    If, for some reason, you want to deactivate the automatic renaming
    feature, it is possible by specifying at the environment level:

    .. code-block:: python

        env = Environment('indoors-1/indoor-1', component_renaming = False)

    If you want to have pymorse working properly without automatic renaming,
    you need to specify name of kind <robot>.<object>

Component properties
--------------------

You can modify the *game-properties* of any components within Python
(or even add new properties). The documentation for each component
lists the game properties it uses, their type and how they affect
the functioning of the component.

For example, to change the resolution of the images captured by a
video camera sensor, modify its properties like this:

.. code-block:: python

    camera = VideoCamera()
    camera.properties(cam_width = 128, cam_height = 128)

.. note::
    You can also add properties this way: if you refer to a property that does
    not exist, the property is created, and become available in other MORSE
    scripts.


Middleware configuration
------------------------

Datastream handlers
+++++++++++++++++++

For usual sensors and actuators, configuring a middleware to access the
component is as easy as::

    motion.add_stream('ros')

One component can be made accessible through several middleware by simply
calling again ``add_stream``::

    motion.add_stream('yarp')

You can check which sensors and actuators are supported by which middleware in
the :doc:`compatibility matrix <integration>`.

.. note::
    Sometimes, you will need to use a specific serialization method.
    This can be achieved by passing more parameters to ``add_stream``::

        motion.add_stream('ros', 'morse.middleware.ros.motion_vw.TwistReader')

    In that case, we instruct MORSE to use ROS with the ``TwistReader`` class
    defined in the :py:mod:`morse.middleware.ros.motion_vw` module.

.. note::
    Configuration for standard sensors and actuators are defined in
    the module :py:mod:`morse.builder.data`.

.. note::
    Some middleware allows you to configure the behaviour of each stream. Refer
    to the documentation of your specific middleware, in the part
    "Configuration specificities" to know more about it.

Service handlers
++++++++++++++++

To use :doc:`services <../dev/services>` of a sensor or an actuator, you
should configure your builder script explicitly.  For example, to export the
service of the actuator ``motion`` through the middleware ``socket``, you must
write::

    motion.add_service('socket')

As for datastream handler, it is possible to configure one component to export
its services through multiple middlewares. You simply need to call
``add_service`` several times.

.. warning::

    Due the nature of some middlewares (in particular ROS or pocolibs), it is
    sometimes not really useful to call the service directly as exposed by
    Morse. You need to use an extra layer of adaption called :doc:`overlays
    <overlays>` and configure it through the ``add_overlay`` method.

Related methods
+++++++++++++++

The method ``add_interface`` allows to configure both datastream and service
handling for one component. So::

    motion.add_stream('socket')
    motion.add_service('socket')

is equivalent to::

    motion.add_inteface('socket')

Last, the method ``add_default_interface`` configures the default interface
for each sensor / actuator owned by a robot. If an interface is configured for
one sensor, it is used, otherwise the default  one is used. In the following
example

.. code-block:: python

    robot = ATRV()

    pose = Pose()
    robot.append(pose)
    pose.add_interface('socket')

    motion = MotionVW()
    robot.append(motion)

    robot.add_default_interface('ros')

``robot.pose`` will be exported through the socket interface, while
``robot.motion`` will be exported through ROS.


Adding modifiers
----------------

Sensors or actuators data can be modifier by assigning modifiers to them.
Modifiers are used to either make some convenient conversions, for instance
when you need to export data related to another frame than the Blender one 
(see the :doc:`UTM <modifiers/utm>` or :doc:`NED <modifiers/ned>` modifiers),
or when you want to add noise to your data.

Modifiers may have parameters (like conversion frame reference, or noise
parameters). They are described in the :doc:`modifiers <modifier_introduction>`
documentation.

To modify the data of a component, just add the following line to 
your builder script::

	pose.alter('Noise', pos_std=0.3)
 

.. _define_new_zone:

Adding a zone
-------------

A zone is a 3d zone, more precisely a rectangular parallelepiped. It is
possible to attach specific properties to each zone, in particular its name
and its type. In the simulator, different behaviours can be implemented. At
the moment, the only Morse component using the concept of zone is the
:doc:`battery <sensors/battery>`.

To add a zone of type ``Charging`` in a scenario, just add the following lines
to your builder script::

    charging_zone_1 = Zone(type = 'Charging')
    # Change its size and move it around (10.0, 0.0, 2.0)
    charging_zone_1.size = [5.0, 5.0, 5.0]
    charging_zone_1.translate(x = 10.0, z = 2.0)

.. _configure_time:

Configuring time in Morse
-------------------------

Time management in simulation is a complex matter: you may want to simulate
different sensors at different speeds, you may want to run
faster-than-real-time simulations, you may want to synchronize the simulator
with an external time reference, etc.

MORSE tries hard to make easy things easy, and complex scenarios possible.

Consider the following simple example:

.. code-block:: python

    from morse.builder import *
    
    # Append ATRV robot to the scene
    robot = ATRV()
    
    # Append an actuator
    motion = MotionVW()
    robot.append(motion)
    
    # Append a sensor
    pose = Pose()
    pose.translate(z = 0.75)
    pose.frequency(200)
    robot.append(pose)
    
    # Configure the robot on the 'socket' interface
    robot.add_default_interface('socket')
    
    env = Environment('indoors-1/indoor-1')

This is the typical scenario: we tell MORSE that the pose sensor should output
values at 200Hz, and we let MORSE manage all other time-related questions. In
this case, MORSE will run its main loop at 200Hz (hardware permitting!
otherwise, MORSE will warn you that the desired frequency can not be reached),
and will attempt to update the physics at real-time speed. Note that we did not
have to specify a frequency for our motion actuator: by default, all MORSE
components run at 60Hz.

Now, imagine you want to accelerate your simulation. Just add the following
lines to your builder script:

.. code-block:: python

    # [...]
    env = Environment('indoors-1/indoor-1')
    env.set_time_scale(1.5)

Now, MORSE will attempt (again, hardware permitting!) to run the simulation at
x1.5 real-time. It means that, during 1 real-time second, Morse will simulation
1.5 second. Accordingly, the pose sensor, for instance, will produce reading at
1.5 X 200 = 300Hz. The physics engine will accordingly run faster, etc.

If you note that sensors do not publish data at a really stable rate, you can
enable **morse_sync**. It may lead to some graphical frame lose, but it will
improve the stability of the sensor rate.

.. code-block:: python

    ...
    env.use_internal_syncer()

Handling more complex scenario
++++++++++++++++++++++++++++++

To handle more complex scenario, you need to read :doc:`this page
<../dev/time_event>` which describes the time handling in Morse. For now, we
have identified two advanced scenarios. If you have specific requirements, or
the default settings cause issues on your computer, pleas report the issue to
the Morse project.

Synchronisation with other(s) simulator(s)
__________________________________________

In this situation, you want to use **Fixed Simulation Step** strategy with
well-defined ``base_frequency``. v-sync can be turned off.

.. code-block :: python

    ...

    env = Env(...)
    env.simulator_frequency(100)
    env.use_vsync('OFF')
    env.set_time_strategy(FixedSimulationStep)
    # in this example, we use HLA, a middleware that is designed to support
    # synchronization amongst heterogeneous systems.
    env.configure_stream_manager('hla', time_sync = True)

Accelerating the simulation by a large factor
_____________________________________________

If you want to accelerate time by a factor of 20 for example, it would
probably be hard (depending your hardware) to provide data at 1200 (60 * 20)
Hz. You may want to lower the frequency of the different components, and
reduce accordingly the ``base_frequency`` of your simulation. As in the
default case, you want to disable v-sync and enable **morse_sync**.


.. code-block :: python

    robot = Morsy()
    robot.frequency(3) 

    pose = Pose()
    robot.append(pose)
    pose.frequency()

    env = Env(...)
    env.simulator_frequency(3)
    env.set_time_scale(20)
    env.use_vsync('OFF')
    env.use_internal_syncer()
