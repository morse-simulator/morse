Adding a  new datastream handler
================================

Datastream handlers allow to transform the internal MORSE model into a
specific serialization type, expected by your specific architecture.

Here, we explain how to add a specific handler for a specific message, in case 
where there is already some infrastructure for your middleware in MORSE

If you want to implement a completely new middleware, refer to
:doc:`new_middleware`.

Adding a new datastream handler : generic part
----------------------------------------------

Adding a new datastream handler is a matter of implementing a new class which
derives from :py:class:`morse.middleware.abstract_datastream.AbstractDatastream`.

The only mandatory method to specialize is the ``default`` method which
contains the actual serialization or deserialization routine. In this method,
you can use ``self.data`` to access to the ``local_data`` field of the
associated component.

It can be interesting too to overriding the ``initialize`` and ``finalize``
method, which contains respectively the initialisation code, and the
finalisation code. Do not override ``__init__`` and ``__del__``.

Last, you can set the ``_type_name`` field, and the ``_type_url`` field to
explicit which type is imported (or exported) with this datastream handler,
and where to find some documentation about it.

Let see an example with a custom socket handler.

.. code-block:: python

    from morse.middleware.abstract_datastream import AbstractDatastream

    class MyPoseExporter(AbstractDatastream):
        _type_name = 'a list x, y, z'

        def initialize(self):
            """ initialization part, creating the socket, binding ... """
            self.s = socket(...)
            # ...

        def finalize(self):
            """ called when we have finished, cleanup resource ... """
            self.s.close()

        def default(self, ci = 'unused'):
            """" place where occur the real encoding """
            my_repr = []
            my_repr.append(self.data['x'])
            my_repr.append(self.data['y'])
            my_repr.append(self.data['z'])
            self.send(repr(my_repr))

Now, you can test your new datastream handler directly in the builder:

.. code-block:: python

    from morse.builder import *

    atrv = ATRV()

    pose = Pose()
    atrv.append(pose)
    pose.add_stream('socket', 'Path.to.MyPoseExporter')

    env = Environment('empty', fastmode=True)


Last, if you want to use it more easily, you can add some entries in
:py:data:`morse.builder.data.MORSE_DATASTREAM_DICT`.

Adding a new datastream handler : specificities
-----------------------------------------------

Generally speaking, the code to send or receive message is the same for all
datastream handler of one middleware, and so it is often shared. In this
section, we outline the specificities of each middleware, which make it easier
to write datastream handler for it.

Text middleware :tag:`text`
+++++++++++++++++++++++++++

Text middleware only supports sensors output. It provides a base class
:py:class:`morse.middleware.text_datastream.BasePublisher` which deals properly
with file handling, name selection, etc. To write a specific text handler, you
want to derive from this base class and reimplement the ``header`` method
(what write in the head of the file) and the ``encode_data`` method (how
to write data in the file). 

.. code-block:: python

    from morse.middleware.text_datastream import BasePublisher

    class MyTextPoseExporter(BasePublisher):
        _type_name = 'a list x, y, z'

        def header(self):
            return 'x, y, z'

        def encode_data(self):
            my_repr = []
            my_repr.append(self.data['x'])
            my_repr.append(self.data['y'])
            my_repr.append(self.data['z'])
            return ', '.join(my_repr)

Socket middleware :tag:`socket`
+++++++++++++++++++++++++++++++

Socket middleware provides two base classes
:py:class:`morse.middleware.socket_datastream.SocketReader` for actuators, and
:py:class:`morse.middleware.socket_datastream.SocketPublisher` for sensors.
These base classes deal in a generic way about socket creation, client
handling, sending and receiving messages. You just need to override the
``decode`` method (respectively ``encode`` method) to provide a custom
encoder.

.. code-block:: python

    from morse.middleware.socket_datastream import SocketPublisher

    class MySocketPoseExporter(SocketPublisher):
        _type_name = 'a list x, y, z'

        def encode(self):
            my_repr = []
            my_repr.append(self.data['x'])
            my_repr.append(self.data['y'])
            my_repr.append(self.data['z'])
            return ', '.join(my_repr)


Yarp middleware :tag:`yarp`
+++++++++++++++++++++++++++

Yarp middleware provide the
:py:class:`morse.middleware.yarp_datastream.YarpPort` which provides basic
encapsulation of the Yarp protocol. A specialized class
:py:class:`morse.middleare.yarp_datastream.YarpPublisher` provides facilities
to send content through a `Yarp::Bottle`. If you want to use such transport,
you can override the method
:py:meth:`morse.middleware.yarp_datastream.YarpPublisher.encode` to provide
specialized behaviour.

.. code-block:: python

    from morse.middleware.yarp_datastream import YarpPublisher

    class MyYarpPoseExporter(YarpPublisher):

        def encode(self, bottle):
            bottle.addString(self.data['x'])
            bottle.addString(self.data['y'])
            bottle.addString(self.data['z'])

Ros middleware :tag:`ros`
+++++++++++++++++++++++++

Ros middleware provides two useful base class
:py:class:`morse.middleware.ros.abstract_ros.ROSReader` and
:py:class:`morse.middleware.ros.abstract_ros.ROSPublisher`, respectively for
actuator and sensor. In particular, they provide some facilities to manage
topics. If you use these classes, you do not need to define ``_type_name`` and
``_type_url``, but to fill ``ros_class``, the previous information will be
derived automatically from it. If you write a Reader, you need to override the
:py:meth:`morse.middleware.ros.abstract_ros.ROSReader.update` method, which
takes a message and must modifier ``self.data`` accordingly. For a Publisher,
you need to override inherited ``default`` method. Don't forget to call
``self.publish(msg)`` otherwise nothing will happen.

.. code-block:: python

    from morse.middleware.ros import ROSPublisher
    from std_msgs.msg import String


    class MyRosPoseExporter(ROSPublisher):
        ros_class = String

        def encode(self):
            my_repr = []
            my_repr.append(self.data['x'])
            my_repr.append(self.data['y'])
            my_repr.append(self.data['z'])
            msg = String(', '.join(my_repr))
            self.publish(msg)

Pocolibs middleware :tag:`pocolibs`
+++++++++++++++++++++++++++++++++++

Pocolibs middleware provides
:py:class:`morse.middleware.pocolibs_datastream.PocolibsDataStreamOutput` for sensors,
and :py:class:`morse.middleware.pocolibs_datastream.PocolibsDataStreamInput` for
actuators which deals with the low-level details of pocolib. To write a custom
encoder, you need to subclass the correct class, and provides both overriding
for ``initialize`` and ``default``. In ``initialize``, do not forget to call
the mother ``initialize`` method which the desired type. In the ``default``
method, do not forget to call respectively ``read`` or ``write``.

.. note::

    Structures imported by pocolibs interface used ctypes. Please read the
    `ctype documentation <http://docs.python.org/3.2/library/ctypes.html>`_
    properly before doing strange things.


.. code-block :: python

    from morse.middleware.pocolibs_datastream import *
    from pom.struct import *

    class MyPocolibsPoseExporter(PocolibsDataStreamOutput)
        _type_name = "POM_POS"

        def initialize(self):
            super(self.__class__, self).initialize(POM_POS)

            # Initialise the object
            self.obj = POM_POS()
            # ...

        def default(self, ci):
            self.obj.mainToOrigin.euler.x = self.data.get('x', 0.0)
            self.obj.mainToOrigin.euler.y = self.data.get('y', 0.0)
            self.obj.mainToOrigin.euler.z = self.data.get('z', 0.0)
            # ...
            self.write(self.obj)


Moos middleware :tag:`moos`
+++++++++++++++++++++++++++

Moos middleware interface provides
:py:class:`morse.middleware.moos.abstract_moos.AbstractMOOS`. This class deals
with some low-level details of Moos. You still need to write your specialized
`default` implementation, calling:

- ``self.m.Notify()`` to export data to the databases (sensor from the point of
  view of Morse)
- ``self.m.FetchRecentMail()`` to get last messages from the databases
  (actuator from the point of view of Morse).

.. code-block:: python

    import pymoos.MOOSCommClient
    from morse.middleware.moos import AbstractMOOS

    class GPSNotifier(AbstractMOOS):
        """ Notify GPS """

        def default(self, ci='unused'):
            cur_time = pymoos.MOOSCommClient.MOOSTime()
            
            self.m.Notify('zEast', self.data['x'], cur_time)
            self.m.Notify('zNorth', self.data['y'], cur_time)
            self.m.Notify('zHeight', self.data['z'], cur_time)
