Creating simulations: the Builder scripts
=========================================

The main and easiest way to create simulations in MORSE is by the way of a
``Builder`` script.

``Builder`` scripts rely on the **Builder API** (described below) to define the
components, environments and middlewaresthat are going to be used in the
simulation. When running MORSE, the simulator interprets the scripts to build
the complete simulation as a Blender scene, configures the components, and starts
the Blender 3D Engine.

Since these scripts are regular Python scripts, you can easily create one
function or class for each of your robots. This way, you can quickly reuse them
in different contexts.

``Builder`` scripts are not the only option to create and/or edit a simulation:
alternatively, you can directly edit from Blender: `Simulation edition in
Blender <advanced_tutorials/editing_in_blender>`.

.. note:: Configuring the environment variables:
    It is necessary to indicate the builder where to look for the installed MORSE
    components in the computer. This is done by specifying the environment variable
    ``$MORSE_ROOT``, which should point to the prefix directory used during
    MORSE installation.

A first example
---------------

First, have a look to :doc:`a Builder script example <builder_example>` to
see how all of this works.

Classes and methods in the Builder API
--------------------------------------

The AbstractComponent base class
++++++++++++++++++++++++++++++++

All the components known to MORSE inherit from the base class
``AbstractComponent``. You should not directly instanciate it.

This class provides the following methods:

 * **name**: change the object name (including the Blender object name)
 * **translate**: The translation will add (x,y,z) to the current object
   location (default: x=0, y=0, z=0, unit: meter).
 * **rotate**: The rotation is an `euler rotation
   <http://www.blender.org/documentation/blender_python_api_2_62_release/bpy.types.Object.html#bpy.types.Object.rotation_euler>`_
   relative to the object's center (default: x=0, y=0, z=0, unit: radian).
 * **properties**: Allows adding/changing the game properties of the Blender
   objects. It receives a list of named items: ``name``=``value``, separated by
   commas.
 * **append**: Add the object given as an argument as a child of this object.
   The argument is an instance to another component. This method is generally
   used to add components to a robot.

This base class has one concrete subclass that can be use to insert :doc:`static
(passive) objects <../user/others/passive_objects>` to your simulation:

 * :py:class:`morse.builder.morsebuilder.PassiveObject`

The Component classes
+++++++++++++++++++++

The class ``Component`` inherits directly from ``AbstractComponent`` and adds
more functions:

 * **add_stream**: Do the binding between a component and the method to
   export/import its data. This must be used in general by sensors and
   actuators. A single component can make several calls to this function to add
   bindings with more than one middleware. The parameter can be either the name
   of the middleware, or a list containing the full path to the middleware
   class and methods that the object will use.
 * **configure_service**: Similar to the previous function. Its argument is the
   name of the middleware to be used.
 * **configure_modifier**: Add a modifier specified by its first argument to
   the current object
 * **configure_overlay**: Add a service overlay for a specific service manager
   (as defined for configure_service) to the current object.
 * **profile**: Watch the average time used by the component during the
   simulation, in percent.

These configuration functions make use of a dictionary defined in the file:
``$MORSE_ROOT/src/morse/builder/data.py``. In these dictionaries, the keys are
the names of the middlewares and the values are the default configurations that
should be written in the ``component_config.py`` file.

There are four subclasses of the ``Component`` class that are used to add
components to a scene.  An instance of these classes must be created to insert
a new component

 * :py:class:`morse.builder.morsebuilder.Robot`
 * :py:class:`morse.builder.morsebuilder.Sensor`
 * :py:class:`morse.builder.morsebuilder.Actuator`
 * :py:class:`morse.builder.morsebuilder.Environment`

.. note::
   When creating instances of these classes, it is necessary to give as
   parameters to the constructors the names of the blender files (without the
   *.blend* extension) that contain the required component. These files should
   be present under ``$MORSE_ROOT/share/morse/data/{class}/``.


The Creator classes
+++++++++++++++++++

Another subclass of ``AbstractComponent`` is ``ComponentCreator``. This class
is used to instantiate components without the need of having a .blend file
associated with them. It is limited to generating components with simple
geometry for their meshes, and limited use of the Logic Bricks in their
behaviour.

As with the regular ``ComponentClass``, there are also a number of subclasses
that inherit from this one:

 * :py:class:`morse.builder.creator.SensorCreator`
 * :py:class:`morse.builder.creator.ActuatorCreator`

The actual definitions of the components that can be instantiated in this way
can be found in the files:
``$MORSE_ROOT/srs/morse/builder/sensors.py`` and
``$MORSE_ROOT/srs/morse/builder/actuators.py``.

To instantiate these kind of objects, you'll need to use the full path of the
class. For example:

.. code-block:: python

  from morse.builder import *
  import morse.builder.sensors
  import morse.builder.actuators

  atrv = ATRV()

  infrared = morse.builder.sensors.Infrared("MyInfrared")
  atrv.append(infrared)

  v_w = morse.builder.actuators.MotionController("MyVOmega")
  atrv.append(v_w)


Environment class
+++++++++++++++++

This is a special case of component that **MUST** be added to a scene.
When an instance of this class is created, it will do several things to properly
configure the scenario to be used in MORSE.

 * Add the background environment where the robots will be tested
 * Configure the general Game Engine settings
 * Write the configuration files for the simulation, based on the configurations done
    for each component
 * Configure the parameters for the :doc:`multi-node <../multinode>` simulation.

The ``Environment`` class provides these functions:

 * **show_framerate**: Toggle the settings in the Game Engine to display
   framerate and profile information of the simulation. The parameter is a
   boolean value indicating whether to show or not this information.
 * **show_physics**: Toggle the display of the bounding boxes of objects during
   the simulation. The parameter is a boolean value indicating whether to show
   or not this information.
 * **show_debug_properties**: Toggle the printing of the value of the Game Properties
   marked. The parameter is a boolean value indicating whether to show or not
   this information.
 * **aim_camera**: Set the orientation of the default camera. The parameter is
   a list with an euler rotation for the camera. Example: *([1.3300, 0,
   0.7854])*
 * **place_camera**: Set the location of the default camera. The parameter is a
   list with the new 3D coordinates for the camera. Example: *([10.0, -10.0,
   3.0])*
 * **set_gravity**: Set the gravity for the specific scene. The parameter is a
   float defaulting to 9.81.
 * **set_viewport**: Set the default view mode in one of 'BOUNDBOX',
   'WIREFRAME', 'SOLID' or 'TEXTURED'
 * **set_debug**: Set the debug bit in blender
 * **set_stereo**: Configure to renderer to render image in 'STEREO' using
   anaglyphs, allowing to see them in 3d with special red-cyan glasses. Allowed
   arguments are:

   - `mode`: Stereographic techniques, enum in ['QUADBUFFERED', 'ABOVEBELOW',
     'INTERLACED', 'ANAGLYPH' (default), 'SIDEBYSIDE', 'VINTERLACE']
   - `eye_separation`: Distance between the eyes. float in [0.01, 5], default 0.1
   - `stereo`: enum in ['NONE' (normal 2D mode), 'STEREO' (default), 'DOME']

 * **select_display_camera**: Indicate to MORSE which camera to display in the
   HUD screen. This method receives as parameter the name of the Builder instance
   of a camera sensor. It will do nothing if the parameter is not a camera.
   The HUD screen can be shown by pressing :kbd:`v` during the simulation
 * **set_horizon_color**: Configure the horizon color to the specified color
   (defined as a triplet R, G, B). See `the blender documentation
   <http://wiki.blender.org/index.php/Doc:2.6/Manual/World/Background>`_ for more
   information about this particular setting.
 * **set_animation_record**: Record the simulation as a Blender animation
   (F-Curves) so you can render it later. See the tutorial: `Recording Game
   Physics to Keyframes <http://cgcookie.com/blender/2011/05/10/tip-recording-game-physics-to-keyframes/>`_
   for more information about this particular setting.
 * **configure_multinode**: Provide the information necessary for the node to
   connect to a multi-node server. The parameter is a list of named items.
   The items accepted in as parameters are:

    * **protocol**: Either 'socket' or 'hla'
    * **server_address**: IP address where the multi-node server can be found
    * **server_port**: Used only for 'socket' protocol. Currently it should always be 65000
    * **distribution**: A Python dictionary. The keys are the names of the
      nodes, and the values are lists with the names of the robots handled by
      each node

   Example:

   .. code-block:: python

        dala1 = ATRV()
        dala2 = ATRV()

        env = Environment('land-1/trees')
        env.configure_multinode(  protocol='socket',
                                  server_address='localhost',
                                  server_port='65000',
                                  distribution={
                                      "nodeA": [dala1.name],
                                      "nodeB": [dala2.name],
                                  })


 * **create()**: Should always be called at the very end of the Builder script.
   It will finalise the building process and write the configuration files.

Note also that the ``configure_service()`` method of the ``Environment`` class
is overloaded: use it to define which middlewares expose the *simulator
internals services* (*i.e.*, the services used to remotely control the
simulator behaviour, cf :doc:`supervision services
<../user/supervision_services>`):

.. code-block:: python

    env = Environement('indoors-1/indoor-1', fastmode = True)
    # Set the simulation management services to be available from ROS:
    env.configure_service("ros")

.. note::
  As seen above, the `Environment` constructor takes an optional parameter `fastmode`: if set to true,
  the simulation is run in wireframe mode, for improved performances. Note that it may be not desirable
  when using video cameras!

Naming of components
--------------------

You can set the name of a component through the setter ``name``::

    mouse = ATRV()
    mouse.name = "jerry"


If you do not explicitely set the name of your components, ``Builder``
scripts rename automatically your components (which includes the Blender
objects representing your components) based on **the name of the variable used
in your Builder script**.

In all cases, the components names are automatically **prefixed with their
parents**, to prevent name collision.

Let take an example. Consider this script, with two robots::

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
    jerry.append(cam)

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

    Automatic renaming only works for components *visible* from your
    script (*ie*, a component declared in a function or class, which is not
    assigned to a variable that belongs to you ``Builder`` script, will not be
    renamed) or components that were appended to a component which is visible.

.. note::
    
    If name collisions occur anyway, Blender automatically adds an
    incremental suffix like ``.001``, ``.002``, etc.

Detailed explanations of class functions
----------------------------------------

Component properties
++++++++++++++++++++

You can modify the game-properties of any components within Python
(or even add new properties). The documentation for each component
lists the game properties it uses, their type and how they affect
the functioning of the component.

For example, to change the resolution of the images captured by a
video camera sensor, modify its properties like this:

.. code-block:: python

    camera = VideoCamera()
    camera.properties(cam_width = 128, cam_height = 128)


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
defined in the file ``src/morse/builder/data.py``). The argument of the 'add_stream'
method is a string with the name of the middleware.

.. code-block:: python

    motion.add_stream('ros')
    motion.add_stream('yarp')

cf. ``morse.builder.data.MORSE_DATASTREAM_DICT``

More than one middleware can be configured for the same component, by using
several calls to the component.add_stream method.

The second one is a bit less simple for the end-user.
It consists of including the description of the middleware binding just as it
would be done by hand in the ``component_config.py`` script:

.. code-block:: python

    motion.add_stream('ros', 'morse.middleware.ros.motion_vw.TwistReader')

cf. :doc:`hooks <../user/hooks>` and the tutorial on :doc:`manually building a scene
<../user/advanced_tutorials/editing_in_blender>` (in particular the section configuring middleware) for details.



