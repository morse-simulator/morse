MORSE-builder API
=================

Abstract
--------

The idea is to allow user to write a simulation in few lines of Python, without 
having to learn Python (neither Blender). We want to give the ease of a DSL, 
without its technical complication.

Code organization
-----------------

The following diagram shows the class hierarchy currently used in MORSE-builder.

.. image:: ../../media/morsebuilder_uml.png
   :align: center 


The point when we want to simplify things, is that we often have to limit the 
freedom of the user. The complexity in simplification is to let user free enough 
to make what he wants without asking him to know every concept behind our 
technologies. The main aim of this API is to build a simulation as simple as 
possible. To do so, we encapsulate the potential complexities of Blender, 
Python or Morse.

To test the API step by step, you can open morse, and in the Blender UI, open 
a Python Console, within which you can import this API 
``from morse.builder.morsebuilder import *``

Configuring the environment variables
+++++++++++++++++++++++++++++++++++++

It is necessary to indicate the builder where to look for the installed MORSE
components in the computer. This is done by specifying the environment variable
``$MORSE_ROOT``, which should point to the prefix directory used during MORSE installation.


Building a full robot
---------------------

Append a robot
++++++++++++++

We can append components by their filename (without the .blend extension)

* to do so, in Blender 2.57+ we just need to have blender-file containing only 
  1 root-object, and as many sub-object as we want, knowing that they'll all be 
  appended.
* in Blender 2.56, it's a bit more tricky, since we don't yet have the 
  `bpy.data.libraries.load 
  <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.BlendDataLibraries.html>`_ 
  method, so we need to generate a components-dictionnary to associate a 
  blender file to its objects. cf. ``morse.builder.data.MORSE_COMPONENTS_DICT``

To append a robot on the scene, we just need to write:

.. code-block:: python

    atrv = Robot('atrv')

In order for this work, there must be a blender-file name atrv.blend in 
the folder ``$MORSE_ROOT/share/data/morse/robots/``.

Append a component
++++++++++++++++++

This task is very similar to the previous one, except that we can make a 
parent relationship with the robot. *ie.*:

.. code-block:: python

    motion = Actuator('v_omega')
    atrv.append(motion)

Once again, this implies that ``v_omega.blend`` exists in 
``$MORSE_ROOT/share/data/morse/actuators/``.

cf. :doc:`<code/morse.builder.html#morse.builder.morsebuilder.Component>`

Move a component
++++++++++++++++

There is 2 way to move a component: ``translate(x,y,z)`` and ``rotate(x,y,z)``.

* The translation will add (x,y,z) the the current object location 
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
(even add them) 

.. code-block:: python

    sick = Sensor('sick')
    sick.properties(resolution = 1)
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

    motion.configure('ros')
    motion.configure('yarp')

cf. ``morse.builder.data.MORSE_MIDDLEWARE_DICT``

More than one middleware can be configured for the same component, by using
several calls to the component.configure method.

The second one is a bit less simple for the end-user.
It consists of including the description of the middleware binding just as it
would be done by hand in the ``component_config.py`` script:

.. code-block:: python

    motion.configure('ros', ['ROS', 'read_twist', 'morse/middleware/ros/read_vw_twist'])

cf. :doc:`user/hooks <../user/hooks>` and :doc:`user/tutorial.html
<../user/tutorial>` (in particular the section configuring middleware)

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
    sick.properties(resolution = 1)

    # Append a camera
    cam = Sensor('video_camera')
    cam.translate(x=0.3,z=1.1)
    atrv.append(cam)
    cam.properties(cam_width = 128, cam_height = 128)

    # Configure the middlewares
    motion.configure('ros')
    odometry.configure('ros')
    proximity.configure('ros')
    pose.configure('ros')
    sick.configure('ros')
    cam.configure('ros')


Generate the components dictionary
-----------------------------------

This part is required for Blender 2.56 developer (if you add new components, 
or want to tweak them)
To do so, you will need Blender 2.57 (I know it doesn't smell usual) since the 
`bpy.data.libraries.load 
<http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.BlendDataLibraries.html>`_ 
method is very convenient to read the content of a blender file.

cf. ``morse.builder.data.MORSE_COMPONENTS_DICT``

cf. ``morse.builder.generator.generate()``

cf. :doc:`<code/morse.builder.html#module-morse.builder.generator>`

TODOs
-----

With this small set of class / proof of concept, we can imagine some tools 
integrated in the Blender GUI to let user append components easily.
