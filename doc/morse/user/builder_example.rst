Using the Builder API
=====================

Let's have a look to a typical ``Builder`` script, and explain it, step by step:

.. code-block:: python
    :linenos:

    from morse.builder import *

    # Append ATRV robot to the scene
    atrv = Robot('atrv')

    # Append an actuator
    motion = Actuator('v_omega')
    motion.translate(z=0.3)
    atrv.append(motion)

    # Append an odometry sensor
    odometry = Sensor('odometry')
    odometry.translate(x=-0.1, z=0.83)
    atrv.append(odometry)

    # Append a proximity sensor
    proximity = Sensor('proximity')
    proximity.translate(x=-0.2, z=0.83)
    atrv.append(proximity)

    # Append a Pose sensor (GPS + Gyroscope)
    pose = Sensor('pose')
    pose.translate(x=0.2,z=0.83)
    atrv.append(pose)

    # Append a sick laser
    sick = Sensor('sick')
    sick.translate(x=0.18,z=0.94)
    atrv.append(sick)

    # Append a camera
    cam = Sensor('video_camera')
    cam.translate(x=0.3,z=1.1)
    atrv.append(cam)
    cam.properties(cam_width = 128, cam_height = 128)

    # Configure the middlewares
    motion.configure_mw('ros')
    odometry.configure_mw('ros')
    proximity.configure_mw('ros')
    pose.configure_mw('ros')
    sick.configure_mw('ros')
    cam.configure_mw('ros')

    env = Environment('land-1/trees')
    env.place_camera([-5.0, 5.0, 3.0])
    env.aim_camera([1.0470, 0, -0.7854])

Components are appended to the scene using the predefined classes
corresponding to the type of the component: ``Robot``, ``Sensor``,
``Actuator``, ``Environment``. The :doc:`Builder API <builder>` page lists all
of them, with their options.

.. note::

    When appending objects to the scene, it is necessary to use the file names
    that contain the Blender object corresponding to your component. For
    :doc:`standard components <../components_library>`, you can omit the path and
    the `.blend` extension. For your own components, give the full path to your
    Blender file.

Append a robot
++++++++++++++

To append a robot on the scene (line 4), we just need to write::

    atrv = Robot('atrv')

In order for this work, there must be a Blender file named ``atrv.blend`` in
the folder ``$MORSE_ROOT/share/morse/data/robots/``. Since ``atrv`` is a
:doc:`standard components <../components_library>`, we omit here the
``.blend`` extension. You could use your own Blender object as well by
specifying a full path.

Append a component
++++++++++++++++++

This task is very similar to the previous one, except that we can make a 
parent relationship with the robot (lines 7 to 9). *ie.*::

    motion = Actuator('v_omega')
    atrv.append(motion)

Once again, this implies that ``v_omega.blend`` exists in 
``$MORSE_ROOT/share/morse/data/actuators/``.

.. note::
    In this example, the motion controller in your simulation will be named
    ``motion``.
    
    The name is used by MORSE to refer to the component in the
    simulator
    interface. Each middleware has it's own naming convention, but for
    instance with the basic ``socket`` interface, you can send a command to
    the motion controller like that::

        $ telnet localhost 4000
        Connected to localhost.
        > req1 motion set_speed [1.0, 0.002]
        req1 OK


Position a component
++++++++++++++++++++

There are 2 transformations you can give to a component: ``translate(x,y,z)`` and ``rotate(x,y,z)``.

* The translation will add (x,y,z) to the current object location 
  (default: x=0, y=0, z=0, unit: meter).
* The rotation is an `euler rotation 
  <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.rotation_euler>`_ 
  relative to the object's center (default: x=0, y=0, z=0, unit: radian).

.. code-block:: python

    motion.translate(x=.2, z=1)
    atrv.rotate(z=3.14)

Component properties
++++++++++++++++++++

You can modify the *game properties* of any components within Python (line
35)::

    sick = Sensor('sick')
    cam = Sensor('video_camera')
    cam.properties(cam_width = 128, cam_height = 128)

.. note::
    You can also add properties this way: if you refer to a property that does
    not exist, the property is created, and become available in other MORSE
    scripts.
  

Middleware configuration
++++++++++++++++++++++++

For usual sensors and actuators, configuring a middleware to access the
component is as easy as::

    motion.configure_mw('ros')

One component can be made accessible through several middleware by simply
calling again ``configure_mw``::

    motion.configure_mw('yarp')

You can check which sensors and actuators are supported by which middleware in
the :doc:`compatibility matrix <integration>`.

.. note::
    Sometimes, you will need to use a specific serialization method.
    This can be achieved by passing more parameters to ``configure_mw``::

        motion.configure_mw(['morse.middleware.ros_mw.ROSClass', 'read_twist', 'morse/middleware/ros/read_vw_twist'])

    In that case, we instruct MORSE to use ROS with the ``read_twist`` method
    defined in the ``morse.middleware.ros.read_vw_twist`` module.

    Refer to :doc:`hooks <../user/hooks>` and the tutorial on :doc:`manually
    building a scene <../user/advanced_tutorials/editing_in_blender>` (in
    particular the section configuring middleware) for details.

.. note::
    Configuration for standard sensors and actuators are defined in 
    the file ``src/morse/builder/data.py``.


Finalising the scene
++++++++++++++++++++

Every builder script must finish with an environment description. This is mandatory, or
else the scene will not be created. The parameter for the *Environment* method is the
name of a *.blend* file that should be located in ``$MORSE_ROOT/share/morse/data/environments/``.

An additional option is to place and aim the default camera, by using the methods *aim_camera* and *place_camera*.

.. code-block:: python

    env = Environment('land-1/trees')
    env.place_camera([-5.0, 5.0, 3.0])
    env.aim_camera([1.0470, 0, -0.7854])



