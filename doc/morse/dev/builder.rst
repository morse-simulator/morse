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

It is necessary to indicate the builder where to look for the installed MORSE components in the computer. This is done by specifying the environment variable ``$MORSE_ROOT``, which should point to the prefix directory used during MORSE installation.


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
the folder ``$MORSE_ROOT/share/data/morse/components/robots/``.

Append a component
++++++++++++++++++

This task is very similar to the previous one, except that we can make a 
parent relationship with the robot. *ie.*:

.. code-block:: python

    motion = Controller('morse_vw_control')
    atrv.append(motion)

Once again, this implies that ``morse_vw_control.blend`` exists in 
``$MORSE_ROOT/share/data/morse/components/controllers/``.

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

    sick = Sensor('morse_sick')
    sick.properties(resolution = 1)
    cam = Sensor('morse_camera')
    cam.properties(cam_width = 128, cam_height = 128)

Middleware configuration
++++++++++++++++++++++++

In order to set a component-middleware-method, we have two options, the first 
one is simple for the user, but require some pre-configuration (dictionary) 
and eventually restriction (middleware-component = 1 method). 

.. code-block:: python

    ros.configure(motion)

cf. ``morse.builder.data.MORSE_MIDDLEWARE_DICT``

The second one is a bit less simple for the end-user.

.. code-block:: python

    ros.configure(motion, ['ROS', 'read_twist', 'morse/middleware/ros/read_vw_twist'])

cf. :doc:`user/hooks <../user/hooks.html>` and :doc:`user/tutorial.html#configuring-the-middlewares <../user/tutorial.html#configuring-the-middlewares>`

Example
-------

.. code-block:: python

    from morse.builder.morsebuilder import *

    # Append ATRV robot to the scene
    atrv = Robot('atrv')

    # Append an actuator
    motion = Controller('morse_vw_control')
    motion.translate(z=0.3)
    atrv.append(motion)

    # Append an odometry sensor
    odometry = Sensor('morse_odometry')
    odometry.translate(x=-0.1, z=0.83)
    atrv.append(odometry)

    # Append a proximity sensor
    proximity = Sensor('morse_proximity')
    proximity.translate(x=-0.2, z=0.83)
    atrv.append(proximity)

    # Append a Pose sensor (GPS + Gyroscope)
    pose = Sensor('morse_pose')
    pose.translate(x=0.2,z=0.83)
    atrv.append(pose)

    # Append a sick laser
    sick = Sensor('morse_sick')
    sick.translate(x=0.18,z=0.94)
    atrv.append(sick)
    sick.properties(resolution = 1)

    # Append a camera
    cam = Sensor('morse_camera')
    cam.translate(x=0.3,z=1.1)
    atrv.append(cam)
    cam.properties(cam_width = 128, cam_height = 128)

    # Insert the middleware object
    ros = Middleware('ros_empty')

    # Configure the middlewares
    ros.configure(motion)
    ros.configure(odometry)
    ros.configure(proximity)
    ros.configure(pose)
    ros.configure(sick)
    ros.configure(cam)


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
