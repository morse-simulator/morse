Adding a new component
======================

Components in MORSE are either **robots**, **sensors** or **actuators**.

**robots** are mainly containers for **sensors** and **actuators**.

This page present how to create and add new sensors or actuators to MORSE.

General overview
----------------

Each component is basically represented by two files, which *must* be created
when adding a new sensor or actuator:

  - A Blender file: has the representation of the sensor on blender, and
    associate the script with the blender object. This file is mandatory, even
    if it contains a single 'Empty' object.
  - A Python script: contains the
    logic of the component

As a starting point, you can :doc:`browse the component
gallery<../components_library>` and look for a component similar to the one you
want to build. You can then use it as a template for our own sensor/actuator.

The 'Blender' part
------------------

  - First, create a nice model of your component.

    - Center it around ``<0,0,0>``
    - 1 Blender unit = 1 m
    - ``x`` points forward, ``z`` points up.
    - If you do not care about giving your object a mesh, simply create a Blender ``Empty`` object.
    - You can of course import meshes in Blender. Just check the scale and orientation.
    - Do not forget that your mesh will be used in a real-time 3D engine: keep
      the number of polygons low ( > 500 for a single model is probably already
      too much. Check the ``decimate`` tool in Blender to simplify your model if
      needed).
    - Do not forget the :doc:`bounding boxes<../user/tips/bounding_boxes>`.
    - If your sensor/actuator has a kinematic structure (not a single rigid part),
      use Blender's armatures to model it precisely.

  - Save the model in ``$MORSE_ROOT/data/<sensors|actuators>/``
  - Press :kbd:`N` to display the properties of the object. Change its name.
  - Press :kbd:`F4` to enter in the logic mode

    - Add the three following *Game properties*:

      - a boolean property named ``Component_Tag`` ((other possible tags are
        ``Robot_Tag``, :doc:`Modifier_Tag<adding_modifier>` and
        ``Middleware_Tag``)) (the value of the property doesn't matter)
      - ``Class`` of type string, which contains the name of the associated
        Python class ``<Sensor>Class`` (e.g. ``GyroscopeClass``)
      - ``Path`` of type string, which contains the path to the associated
        Python script (within the ``src/`` directory, and without the trailing
        ``.py``): ``morse/sensors/<Sensor>``  (e.g.
        ``morse/sensors/gyroscope``)

The names specified in the **Path** and **Class** properties must match exactly
the location of the Python file and the name of the defined class,
respectively. The information in these variables will be used to dynamically
load the module and class during initialisation of the simulation.

Also make sure that the `Parent Inverse
<http://wiki.blender.org/index.php/User:Pepribal/Ref/Appendices/ParentInverse>`_
is identity, otherwise your sensor might have an offset when parented to your
robot even if you specified zero as location.

You can inspect this matrix from the python console:
    ``bpy.data.objects['your_object_name'].matrix_parent_inverse``
And set it to identity again if needed:
    ``bpy.data.objects['your_object_name'].matrix_parent_inverse.identity()``

The Python part
---------------

You need to implement a sub-class of ``morse.core.sensor.MorseSensorClass``, 
respectively of ``morse.core.actuator.MorseActuatorClass``

Important things to do :

  - in the constructor of the object (``__init__``), initialize each variable
    you want to expose to the world (for sensors) or read (for actuators) into
    ``local_data``. Have a look for example to the `PR2_posture sensor
    <https://github.com/laas/morse/blob/master/src/morse/sensors/pr2_posture.py#L15>`_
    source.

  - override ``default_action`` : it must contains the logic of our component.
    Avoid to do some big computation here : the function is called often, and
    it will slow down the whole processing of the Game Engine.

      - For a sensor, you want to **create** there data to be exported by the
        simulator. See for instance the ``default_action`` method of the
        `PR2_posture sensor
        <https://github.com/laas/morse/blob/master/src/morse/sensors/pr2_posture.py#L121>`_.
      - For an actuator, you want to **modify** the simulated scene based on
        the values stored in the ``local_data`` dictionary. Have a look to the
        `Pan Tilt Unit actuator
        <https://github.com/laas/morse/blob/master/src/morse/actuators/ptu.py#L143>`_,
        for instance.

.. note::
    Note that you never directly discuss with a middleware inside a component.
    Everything goes through the ``local_data`` structure. This lets your code
    be largely middleware independant.

    To put it another way: your component **must not** have any middleware
    specific code.

Getting data or exporting data
------------------------------

A component is not really useful if it doesn't get any input (for an actuator)
or if you can't use the output of a sensor. You can use different middleware to
import / export data. 

In the simplest case, you can use automatic serialization, which will try to
convert the data in ``local_data`` OrderedDict into the appropriate format to send
through the middleware. This works only for the basic data types of integer,
float or string.  If you want more specific behaviour for other data types, you
need to add a method to the middleware provider of your choice (for example, if
you want to export a new sensor through YARP, you need to add a method to
MorseYarpClass, in ``$MORSE_ROOT/src/morse/middleware/yarp_mw.py``). The method
must have the following prototype :::

  def your_method(self, component_instance):

For instance, a specific serialization method has been defined to serialize
RGBA images for YARP :::

  def post_image_RGBA(self, component_instance):
	""" Send an RGBA image through the given named port."""
	#...formatting the sensor data stored in component_instance.local_data
	yarp_port.write()

(see ``$MORSE_ROOT/src/morse/middleware/yarp_mw.py`` for the complete method)

In this method, you can access / store component information through its dictionary
``local_data``. In case of a sensor, it is not expected that you change the
content of the sensor, but only read information in this array.

After that, you need to register your new function into the middleware
abstraction.  For that, you need to modify the method ``register_component``.
It is basically a switch case with the different possible functions. This
method is called when parsing the configuration file for the scene, so
it is the right place to initialize stuff (opening Yarp ports, sockets, files
...)

Middleware specific information
-------------------------------

YARP
____

In MorseYarpClass, the different port_name are stored in a dictionary
``_component_ports``, indexed by the name of the component
(``component.blender_obj.name``). You can retrieve the associated port with the
method ``getPort(port_name)``

Example: ::

    port_name = self._component_ports[component_instance.blender_obj.name]

    try:
	    yarp_port = self.getPort(port_name)
    except KeyError as detail:
	    print ("ERROR: Specified port does not exist: ", detail)
	    return


Pocolibs
________

In MorsePocolibsClass, the different poster_id are stored in a dictionary
``_poster_dict``, indexed by the name of the component
(``component.blender_obj.name``)

Text
____

In TextOutClass, the different files are stored in a dictionary
``_file_list``, indexed by the name of the component
(``component.blender_obj.name``)
