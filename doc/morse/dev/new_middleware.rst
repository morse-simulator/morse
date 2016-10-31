Adding support for a new middleware to MORSE
============================================

Adding a datastream manager
---------------------------

When you want to add a new middleware to MORSE, you must first create a
'datastream' manager. Basically, its role is to make the link between components
and a datastream handler. From an implementation point of view, it must be a
subclass of the :py:class:`morse.core.datastream.DatastreamManager`. In addition to
reimplementing the ``__init__`` and ``__del__`` methods, you may want to override the
:py:meth:`morse.core.datastream.DatastreamManager.register_component`  method which
actually includes the registration logic. However, in general, the default
implementation should be adequate. You can find some examples of overriding in
:py:meth:`morse.middleware.socket_datastream.Socket.register_component` where we
store additional information to cater for different services.

If you need to run some general datastream/middleware code once per simulation
loop, you can also override the :py:meth:`morse.core.datastream.DatastreamManager.action`` method.

Module Organisation
-------------------

To match the MORSE organisation, we expect the following file hierarchy:

- in morse.middleware, a file ``<your_middleware>_datastream.py`` which
  contains the datastream manager for your middleware.
- in morse.middleware.your_middleware, a set of files implementing
  different datastream handlers (see :doc:`adding_datastream_handler`) for
  different messages. It is recommended that you abstract generic handling of
  your middleware into two base classes (Publisher and Reader).

The builder part
----------------

To facilitate the use of your middleware, you can add some entries in
:py:mod:`morse.builder.data`. In particular, you need to add an entry for your
datastream manager in :py:data:`morse.builder.data.MORSE_DATASTREAM_MODULE`, and
add the necessary entries for the different datastream handlers in
:py:data:`morse.builder.data.MORSE_DATASTREAM_DICT`.
