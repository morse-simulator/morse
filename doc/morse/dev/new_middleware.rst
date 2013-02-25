Creation of new middlewares
===========================

To be able to create new middleware bindings for MORSE, it is necessary to
understand the concept of :doc:`hooks <../user/hooks>`, which describes how
data can be accessed from the MORSE components and passed to the middleware
bindings.

The easiest way to add middleware communications to MORSE is to take a look
at the existing modules, copy one of them, and modify it according to
your requirements.

Probably the best example for this (at the same time simple and complete) is
the :doc:`YARP <../user/middlewares/yarp>` middleware.

The Python script
-----------------

- Copy one of the existing ``${middleware}_datastream.py`` Python files in
  ``$MORSE_SRC/src/morse/middleware``
- Change the name of the class, and write down its behaviour, implementing
  the ``register_component`` function
- The function ``register_component`` must assign the serialization function
  to either the list of ``input_functions`` or ``output_functions`` of the
  component instance, according to its type
- Implement the actual serialization / deserialization function, that will
  convert between the information in the ``local_data`` variable of the
  component and the format required for transmission by the middleware

Basic data serialization
++++++++++++++++++++++++

If you have a few different messages to decode / encode, it is possible to
implement everything in the single file ``${middleware}_datastream.py``.
This is normally the case when you want to transmit data of the basic types:
integer, float, string, boolean.

The ``register_component`` method must make sure to assign the correct
function to the MORSE component, depending on the type of component and the
data to be transmitted.


Advanced data serialization
+++++++++++++++++++++++++++

For more specific data types that require a special encoding / decoding,
it is preferable to split each processing in a separate file.
For this purpose, you need to create a directory ``$MORSE_SRC/src/morse/${middleware}``.
Each file will implement a special processing for a given type of message.
These files need to implement at least two different methods:

- ``init_extra_module(self, component_instance, function, mw_data)``: this
  method will register the new serializer to its associated middleware
- The method to really serialize / deserialize the component input / output.

As an example, look at the file ``$MORSE_SRC/src/morse/middleware/yarp/sick.py``.
The methods implemented here convert the list of lists stored in ``local_data``
of the :doc:`sick sensor <../user/sensors/laserscanner>` into a yarp bottle with a
series of nested bottles for each point, each bottle containing three double
variables, for the x, y and z coordinates.

When you write a method which serializes the component output, you must not
modify the associated component. You are allowed to make computations to adapt
the internal MORSE data structure (``local_data``) to the needs of your
specific middleware (it is the main goal of this layer in fact).
If you feel that some sensor / actuator requires
more information, you must add (propose a patch, or inherit and extend) it at
the sensor level and not at the middleware level, so every potential user
can profit from it (and not only people using this specific middleware).

Getting data or exporting data
++++++++++++++++++++++++++++++

A component is not really useful if it doesn't get any input (for an actuator)
or if you can't use the output of a sensor. You can use different middleware to
import / export data. 

In the simplest case, you can use automatic serialization, which will try to
convert the data in ``local_data`` OrderedDict into the appropriate format to send
through the middleware. This works only for the basic data types of integer,
float or string.  If you want more specific behaviour for other data types, you
need to add a method to the middleware provider of your choice (for example, if
you want to export a new sensor through YARP, you need to add a method to
Yarp, in ``$MORSE_ROOT/src/morse/middleware/yarp_datastream.py``). The method
must have the following prototype :::

  def your_method(self, component_instance):

For instance, a specific serialization method has been defined to serialize
RGBA images for YARP :::

  def post_image_RGBA(self, component_instance):
    """ Send an RGBA image through the given named port."""
    #...formatting the sensor data stored in component_instance.local_data
    yarp_port.write()

(see ``$MORSE_ROOT/src/morse/middleware/yarp_datastream.py`` for the complete method)

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
++++

In Yarp, the different port_name are stored in a dictionary
``_component_ports``, indexed by the name of the component
(``component.bge_object.name``). You can retrieve the associated port with the
method ``getPort(port_name)``

Example: ::

    port_name = self._component_ports[component_instance.bge_object.name]

    try:
        yarp_port = self.getPort(port_name)
    except KeyError as detail:
        print ("ERROR: Specified port does not exist: ", detail)
        return


Pocolibs
++++++++

In Pocolibs, the different poster_id are stored in a dictionary
``_poster_dict``, indexed by the name of the component
(``component.bge_object.name``)

Text
++++

In Text, the different files are stored in a dictionary
``_file_list``, indexed by the name of the component
(``component.bge_object.name``)
