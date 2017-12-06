Adding a  new datastream handler
================================

Datastream handlers allow you to transform the internal MORSE model into the
serialization form expected by your specific architecture.

Here, we explain how to add a custom handler for a specific message, for when
there is already some infrastructure for your middleware in MORSE.

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

It can also be useful to override the ``initialize`` and ``finalize``
methods, which contain the initialisation code, and the
finalisation code. Do not override ``__init__`` and ``__del__``.

Lastly, you can set the ``_type_name`` field, and the ``_type_url`` field to
specify which type is imported (or exported) with this datastream handler,
and where to find some documentation about it.

Let's see an example with a custom socket handler.

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


Lastly, if you want to use it more easily, you can add some entries in
:py:data:`morse.builder.data.MORSE_DATASTREAM_DICT`.

Adding a new datastream handler : specifics
-------------------------------------------

Generally speaking, the code to send or receive a message is the same for all
of a particular middleware's datastreams, and so it is often shared. In this
section, we outline the specifics of each middleware, which make it easier
to write a datastream handler for it.

Text middleware :tag:`text`
+++++++++++++++++++++++++++

Text middleware only supports sensors output. It provides a base class
:py:class:`morse.middleware.text_datastream.BasePublisher` which deals properly
with file handling, name selection, etc. To write a specific text handler, you
want to derive from this base class and reimplement the ``header`` method
(what to write in the head of the file) and the ``encode_data`` method (how
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

Yarp middleware includes
:py:class:`morse.middleware.yarp_datastream.YarpPort` which provides basic
encapsulation of the Yarp protocol. A specialized class
:py:class:`morse.middleware.yarp_datastream.YarpPublisher` provides facilities
to send content through a `Yarp::Bottle`. If you want to use this transport,
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

Ros middleware provides two useful base classes:
:py:class:`morse.middleware.ros.abstract_ros.ROSSubscriber` for actuators, and
:py:class:`morse.middleware.ros.abstract_ros.ROSPublisher` for
sensors. In particular, they provide some facilities to manage
topics. If you use these classes, you do not need to define ``_type_name`` or
``_type_url``, but instead must set ``ros_class``, from which the ``_type_name`` and
``_type_url`` will automatically be derived. If you write a Reader, you need to override the
:py:meth:`morse.middleware.ros.abstract_ros.ROSSubscriber.update` method, which
takes a message and must modify ``self.data`` accordingly. For a Publisher,
you need to override the inherited ``default`` method. Don't forget to call
``self.publish(msg)`` otherwise nothing will happen.

.. code-block:: python

    from morse.middleware.ros import ROSPublisher
    from std_msgs.msg import String


    class MyRosPoseExporter(ROSPublisher):
        ros_class = String

        def default(self, ci='unused'):
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
actuators, and which deal with the low-level details of Pocolibs. To write a custom
encoder, you need to subclass the correct class, and override both the
``initialize`` and ``default`` methods. In ``initialize``, remember to call
the parent class' ``initialize`` method with the desired type. In the ``default``
method, remember to call ``read`` or ``write`` as appropriate.

.. note::

    Structures imported by the pocolibs interface use ctypes. Please read the
    `ctype documentation <http://docs.python.org/3.2/library/ctypes.html>`_
    properly to avoid strange things happening.


.. code-block :: python

    from morse.middleware.pocolibs_datastream import *
    from pom.struct import *

    class MyPocolibsPoseExporter(PocolibsDataStreamOutput)
        _type_name = "POM_POS"

        def initialize(self):
            PocolibsDataStreamOutput.initialize(self, POM_POS)

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

Moos middleware interface provides two base classes:
:py:class:`morse.middleware.moos.abstract_moos.MOOSNotifier`, and
:py:class:`morse.middleware.moos.abstract_moos.MOOSSubscriber`.
The first ``MOOSNotifier`` can be used for sensors to notify `MOOSDB` of
new data,
the second ``MOOSSubscriber`` can be used for actuators to receive data from the
`MOOSDB`.

When writing a ``MOOSNotifier`` subclass, you need to override the ``default()``
method to do the publications:

.. code-block:: python

    from morse.middleware.moos import MOOSNotifier

    class ExampleNotifier(MOOSNotifier):
        """ Example of MOOSNotifier """

        def default(self, ci='unused'):
            self.notify('MORSE_TEST', 'true')

When writing a ``MOOSSubscriber`` subclass, you need to override the
``initialize()`` method to be able to register properly and create the proper
callback:

.. code-block:: python

    from morse.middleware.moos import MOOSSubscriber

    class ExampleSubscriber(MOOSSubscriber):
        """ Example of MOOSSubscriber """

        def initialize(self):
            # initialize the parent class
            MOOSSubscriber.initialize(self)
            # register and set callback
            self.register_message_to_queue('MORSE_EXAMPLE',
                              'example_queue', self.on_msg)

        def on_msg(self, msg):
            logger.info('ExampleSubscriber.on_msg received %s with val= %s'%(
                  msg.key(), msg.string()))

HLA middleware :tag:`hla`
+++++++++++++++++++++++++

HLA middleware provides several facilities, i.e.
:py:class:`morse.middleware.hla.abstract_hla.AbstractHLAOutput` for sensors, and
:py:class:`morse.middleware.hla.abstract_hla.AbstractHLAInput` for actuators, to
deal with common HLA stuff. Important for HLA datastream handlers is the
``_hla_name``, which defines the name desired / expected from the federation.
The default name is the robot parent name. Otherwise, the structure of a
HLA actuator datastream handler looks like:

.. code-block:: python

    from morse.middleware.hla.message_buffer import MessageBufferReader
    from morse.middleware.hla.abstract_hla import AbstractHLAInput

    class CertiTestInput(AbstractHLAInput):
        def initialize(self):
            super().initialize()

            # Grab handler to objects and attribute handles
            bille_handle = self.amb.object_handle('Bille')

            self.handle_x = self.amb.attribute_handle("PositionX", bille_handle)
            self.handle_y = self.amb.attribute_handle("PositionY", bille_handle)

            self.suscribe_attributes(bille_handle, [self.handle_x, self.handle_y])

        def default(self, ci = 'unused'):
            attributes = self.get_attributes()

            # Check if we receives attributes
            if attributes and attributes[self.handle_x] and attributes[self.handle_y]:
                # Decode the attributes
                x = MessageBufferReader(attributes[self.handle_x]).read_double()
                y = MessageBufferReader(attributes[self.handle_y]).read_double()
                ...

On the other side, the initialize is quite symmetric (i.e. get handles over
objects and attributes, and then publish them. The construction of the output
is done in the following way:

.. code-block:: python

    class CertiTestOutput(AbstractHLAOutput):
        def default(self, ci = 'unused'):
            to_send = {self.handle_x: MessageBufferWriter().write_double(self.data['x']),
                       self.handle_y: MessageBufferWriter().write_double(self.data['y'])}
            self.update_attribute(to_send)

Mavlink middleware :tag:`mavlink`
+++++++++++++++++++++++++++++++++

Morse provides facilities to help the interoperability with Mavlink, with the
classes :py:class:`morse.middleware.mavlink.abstract_mavlink.MavlinkSensor`
and :py:class:`morse.middleware.mavlink.abstract_mavlink.MavlinkActuator`.
These classes provides a generic ``default`` implementation, so you just need
to override the method ``make_msg`` (for sensors) or ``process_msg`` (for
actuators). These methods must generate (or read) the ``self._msg`` field,
which contains a message using the Mavlink protocol. It is also important (for
documentation purposes) to provide the ``_type_name`` for the class,
corresponding to the name of the associated Mavlink type. See for example the
implementation for the attitude Mavlink message:

.. code-block:: python

    from morse.middleware.mavlink.abstract_mavlink import MavlinkSensor
    from pymavlink.dialects.v10 import common as mavlink

    class AttitudeSensor(MavlinkSensor):
        _type_name = "ATTITUDE"

        def make_msg(self):
            # Expects the coordinate in aeronautical frame
            self._msg = mavlink.MAVLink_attitude_message(
                    self.time_since_boot(),
                    self.data['rotation'][0],
                    - self.data['rotation'][1],
                    - self.data['rotation'][2],
                    self.data['angular_velocity'][0],
                    - self.data['angular_velocity'][1],
                    - self.data['angular_velocity'][2]
            )

PPRZLink middleware :tag:`pprzlink`
+++++++++++++++++++++++++++++++++++

The support of PPRZLink is very similar to the one of Mavlink describe above.
It provides the two classes
:py:class:`morse.middleware.pprzlink.abstract_pprzlink.PprzlinkSensor`
and :py:class:`morse.middleware.pprzlink.abstract_pprzlink.PprzlinkActuator`.
These classes provides a generic ``default`` implementation, so you just need
to override the method ``make_msg`` (for sensors) or ``process_msg`` (for
actuators). These methods must generate (or read) the ``self._msg`` field.
It is necessary to provide the ``_type_name`` field for reading objects as it
is used for binding to the correct PPRZLink message. See for example the
implementation for receiving the ROTORCRAFT_FP message used to set the
position of a quadrotor in an environment:

.. code-block:: python

    from morse.middleware.pprzlink.abstract_pprzlink import PprzlinkActuator
    from pprzlink.message import PprzMessage 
    
    # convenience function to convert integer position value to float
    def pos_of_int(pos):
        return float(pos) / 2**8
    
    # convenience function to convert integer angle value to float
    def angle_of_int(angle):
        return float(angle) / 2**12
    
    """ Set rotorcraft pose using the Teleport actuator """
    class RotorcraftPose(PprzlinkActuator):
        _type_name = "ROTORCRAFT_FP"
    
        def process_msg(self):
            # the actuator assumes ned control, so don't do any transformation
            self.data['x'] = pos_of_int(self._msg['east'])
            self.data['y'] = pos_of_int(self._msg['north'])
            self.data['z'] = pos_of_int(self._msg['up'])
            self.data['roll']   = angle_of_int(self._msg['phi'])
            self.data['pitch']  = angle_of_int(self._msg['theta'])
            self.data['yaw']    = angle_of_int(self._msg['psi'])

