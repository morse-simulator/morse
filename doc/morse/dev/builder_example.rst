Using the Builder API
=====================

Components are appended to the scene using the predefined classes corresponding
to the type of the component:

 * Robot
 * Sensor
 * Actuator
 * Environment

.. note:: When appending objects to the scene, it is necessary to use the
    file names that contain the components in Blender (without the *.blend* extension)

Append a robot
++++++++++++++

To append a robot on the scene, we just need to write:

.. code-block:: python

    atrv = Robot('atrv')

In order for this work, there must be a blender-file name atrv.blend in 
the folder ``$MORSE_ROOT/share/morse/data/robots/``.

Append a component
++++++++++++++++++

This task is very similar to the previous one, except that we can make a 
parent relationship with the robot. *ie.*:

.. code-block:: python

    motion = Actuator('v_omega')
    atrv.append(motion)

Once again, this implies that ``v_omega.blend`` exists in 
``$MORSE_ROOT/share/morse/data/actuators/``.

cf. :py:mod:`morse.builder.morsebuilder.Component`

Place a component
+++++++++++++++++

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

You can modify the game-properties of any components within Python 
(or even add new ones).

.. code-block:: python

    sick = Sensor('sick')
    cam = Sensor('video_camera')
    cam.properties(cam_width = 128, cam_height = 128)

Middleware configuration
++++++++++++++++++++++++

The builder script also permits creating the required ``component_config.py``
for the scene according to the robot and components being inserted. This is
done automatically so that the user does not need to modify said script by
hand.

The middleware controllers required by the configuration will be automatically
added to the scene when the builder script is parsed.

In order to set a component-middleware-method, we have two options, the first
one is simple for the user, but requires some pre-configuration (a dictionary
defined in the file ``src/morse/builder/data.py``). The argument of the 'configure'
method is a string with the name of the middleware.

.. code-block:: python

    motion.configure_mw('ros')
    motion.configure_mw('yarp')

cf. ``morse.builder.data.MORSE_MIDDLEWARE_DICT``

More than one middleware can be configured for the same component, by using
several calls to the component.configure method.

The second one is a bit less simple for the end-user.
It consists of including the description of the middleware binding just as it
would be done by hand in the ``component_config.py`` script:

.. code-block:: python

    motion.configure_mw(['morse.middleware.ros_mw.ROSClass', 'read_twist', 'morse/middleware/ros/read_vw_twist'])

cf. :doc:`hooks <../user/hooks>` and the tutorial on :doc:`manually building a scene
<../user/advanced_tutorials/editing_in_blender>` (in particular the section configuring middleware) for details.


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



Example
-------

.. code-block:: python

    from morse.builder.morsebuilder import *

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
