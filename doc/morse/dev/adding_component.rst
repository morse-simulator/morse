Adding a new component
======================


Components in MORSE are either **robots**, **sensors** or **actuators**.

**Robots** are mainly containers for **sensors** and **actuators**.

This page present how to create and add new sensors or actuators to MORSE.

General overview
----------------

A component is mainly described by a Python script, which contains its logic.
It may also have a related Blender file, which contains its appearance, and in
some case, some specific logic. In general, it is not needed. Last, you may
want to add some entries in builder to facilitate the use of your component.

As a starting point, you can :doc:`browse the component
gallery<../components_library>` and look for a component similar to the one you
want to build. You can then use it as a template for our own sensor/actuator.

The Python part
---------------

You need to implement a sub-class of :py:class:`morse.core.sensor.Sensor`, 
respectively of :py:class:`morse.core.actuator.Actuator`.

Let start to write a custom sensor to read image. First, you must define the
``_name`` field and the ``_short_desr`` field to describe your component.

.. code-block:: python

    from morse.core.sensor import Sensor

    class MyImageSensor(Sensor):
        
        _name = "MyImageSensor"
        _short_descr = "A custom Image Sensor"


Defining exported data fields
+++++++++++++++++++++++++++++

Components import (for actuators) or export (for sensors) data. These data are
declared as a set of *data field* specific to the component, accessible
through the ``local_data`` map. You must use the
:py:meth:`morse.helpers.components.add_data` to declare the different fields
of your component.

.. code-block:: python

    from morse.core.sensor import Sensor
    from morse.helpers.components import add_data

    class MyImageSensor(Sensor):

        _name = "MyImageSensor"
        _short_descr = "A custom Image Sensor"

        add_data("image", None, 'rgba buffer', 'the data captured by the ImageSensor, stored as a Python Buffer ...')
        add_data('matrix', None, "mat3<float>", 'long description')

Defining properties for your component
++++++++++++++++++++++++++++++++++++++

It is possible to define properties for your components, i.e. some variables
which allow to configure your component. For example, you may want to set the
size of your image. To declare such properties, you need to use the
:py:meth:`morse.helpers.components.add_property`. 

.. code-block:: python

    from morse.core.sensor import Sensor
    from morse.helpers.components import add_data, add_property

    class MyImageSensor(Sensor):

        _name = "MyImageSensor"
        _short_descr = "A custom Image Sensor"

        add_data("image", None, 'rgba buffer', 'the data captured by the ImageSensor, stored as a Python Buffer ...')
        add_data('matrix', None, "mat3<float>", 'long description')

        add_property('image_width', 256, 'image_width', 'int', 'width of the image, in pixel')
        add_property('image_length', 256, 'image_length', 'int', 'width of the image, in pixel')

.. warning::

    Contrary to ``add_data``, you must only pass basic type in
    ``add_property`` (bool, float, int, string). Indeed, her, we rely on the
    blender game property system to pass values between the builder script and
    the code logic, and it only supports these basic types.

Defining the logic of your component
++++++++++++++++++++++++++++++++++++

Now that we have defined the interface of our component, we need to define its
internal logic. There are two important functions that you want to override.

  - the init function (``__init__``). In this function, you can create and
    initialize private attributes (which won't be exported to other MORSE
    layer). Do not forget to call the ``__init__`` method of your mother
    class, to properly initialize the component.

  - the ``default_action`` method contains the logic of our component.  Avoid
    to do some big computation here: the function is called often, and it will
    slow down the whole processing of the Game Engine.

      - For a sensor, you want to compute the values of the different elements
        of your ``local_data`` using the current simulator step. See for
        instance :py:meth:`morse.sensors.pose.Pose.default_action`.

      - For an actuator, you want to **modify** the simulated scene based on
        the values stored in the ``local_data`` dictionary. Have a look at
        :py:meth:`morse.actuators.v_omega.MotionVW.default_action` for
        instance.

.. code-block:: python

    from morse.core.sensor import Sensor
    from morse.helpers.components import add_data, add_property

    class MyImageSensor(Sensor):

        _name = "MyImageSensor"
        _short_descr = "A custom Image Sensor"

        add_data("image", None, 'rgba buffer', 'the data captured by the ImageSensor, stored as a Python Buffer ...')
        add_data('matrix', None, "mat3<float>", 'long description')

        add_property('image_width', 256, 'image_width', 'int', 'width of the image, in pixel')
        add_property('image_length', 256, 'image_length', 'int', 'width of the image, in pixel')

        def init(self, obj, parent = None):
            # Call the constructor of the parent class
            Sensor.__init__(self, obj, parent)

            # Initialize some private variable
            self.capturing = False
            # ...

            # Initialize some field of local_data
            self.local_data['matrix'] = mathutils.Matrix()
            # ...

            # Inform the user that everything is fine
            logger.info('Component initialized')

        def get_raw_image(self):
            #...

        def default_action(self):
            self.local_data["image"] = self.get_raw_image()

.. note::
    Note that you never directly discuss with a middleware inside a component.
    Everything goes through the ``local_data`` structure. This lets your code
    be largely middleware independant.

    To put it another way: your component **must not** have any middleware
    specific code.

.. note::
    
    You may want to add services to your component. Please follow
    :doc:`services` to learn how to add service to one component.


Defining abstraction levels
+++++++++++++++++++++++++++

A component can define several levels of abstraction, also called levels of
*realism*. One interesting example is the :py:mod:`morse.sensors.odometry`
which defines three levels of realism, corresponding to different degrees of
integration.

These levels consist in:

  - a custom set of data fields,
  - and/or a custom component class implementation.

Levels are defined with the helper function
:py:meth:`morse.helpers.components.add_level`. The function
:py:meth:`morse.helpers.components.add_data` can take an extra argument, which
represents the level of the data (data will appears only at this level of
realism). If not present, the data is available to all realism level.

.. code-block:: python

    from morse.core.sensor import Sensor
    from morse.helpers.components import add_level, add_data

    class MyImageSensor(Sensor):
        """ This imaginary image sensor can either provide 'raw' images,
        or denoised images.
        """

        # We define 2 levels for this sensor:
        add_level("raw", None, "provides raw data")
        add_level("processed", "path.to.my.MyProcessedImageSensor", "provides cleaned images", default=True)

        add_data("image", None, "rgba", "raw image", level = "raw")
        add_data("image", None, "rgba", "denoised image", level = "processed")
        add_data("noise_level", None, "float", "level of removed noise", level = "processed")

        #add a constructor...

        def get_raw_image(self):
            #...

        def default_action(self):
            self.local_data["image"] = self.get_raw_image()

     class MyProcessedImageSensor(MyImageSensor):

        #add a constructor...

        def clean_image(self, image):
            # ...

        def default_action(self):
            image = self.get_raw_image
            cleaned, level = self.clean_image(image)

            self.local_data["image"] = cleaned
            self.local_data["noise_level"] = level


Here, we define two level of realism, the `raw` one and the `processed`
one. The `raw` level is implemented directly by `MyImageSensor` while the
`processed` level is handled by `MyProcessedImageSensor` class.

We may observe that the `processed` level as a flag `default=True`. While not
mandatory, it is recommended to define a default level to allow the usage of
your component with minimal configuration.

An user would configure this sensor in a script that way:

.. code-block:: python

    from morse.builder import *

    robot = ATRV()

    image = MyImageSensor()
    image.level("processed")
    robot.append(image)

    ...



The 'Blender' part
------------------

  - First, create a nice model of your component.

    - Center it around ``<0,0,0>``
    - 1 Blender unit = 1 m
    - ``x`` points forward, ``z`` points up.
    - You can of course import meshes in Blender. Just check the scale and orientation.
    - Do not forget that your mesh will be used in a real-time 3D engine: keep
      the number of polygons low ( > 500 for a single model is probably already
      too much. Check the ``decimate`` tool in Blender to simplify your model if
      needed).
    - Do not forget the :doc:`bounding boxes<../user/tips/bounding_boxes>`.
    - If your sensor/actuator has a kinematic structure (not a single rigid part),
      use Blender's armatures to model it precisely.

  - Save the model in ``$MORSE_ROOT/data/<sensors|actuators>/``

Make sure that the `Parent Inverse
<http://wiki.blender.org/index.php/User:Pepribal/Ref/Appendices/ParentInverse>`_
is identity, otherwise your sensor might have an offset when parented to your
robot even if you specified zero as location.

You can inspect this matrix from the python console:
    ``bpy.data.objects['your_object_name'].matrix_parent_inverse``
And set it to identity again if needed:
    ``bpy.data.objects['your_object_name'].matrix_parent_inverse.identity()``

Specific case of armatures
++++++++++++++++++++++++++

**Armatures** are the MORSE way to simulate kinematic chains made of a
combination of revolute joints (hinge) and prismatic joints (slider).

They require special care to be successfully crafted. Please refer
to the :doc:`armature creation<armature_creation>` page for details.


The Builder Part
----------------

.. warning::

    TODO : write this part :)
