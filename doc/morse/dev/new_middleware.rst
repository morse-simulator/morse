Adding the support for a new middleware to MORSE
================================================

Adding a datastream manager
---------------------------

When you want to add a new middleware to MORSE, you need first to create a
'datastream' manager. Basically, its role is to make the link between
components and datastream handler. From an implementation point of view, it
must be a subclass or :py:class:`morse.core.datastream.Datastream`. In addition
to classic method ``__init__`` and ``__del__``, you may want to override the
method :py:meth:`morse.core.datastream.Datastream.register_component` which
really includes the logic of registration. However, in general, the default
implementation is enough. You can find some example of overloading in
:py:meth:`morse.middleware.socket_datastream.Socket.register_component`
where we store additional informations for the sake of different services.

Module Organisation
-------------------

To match the MORSE organisation, we expect the following file hierarchy:

    - in morse.middleware, a file ``<your_middleware>_datastream.py`` which
      contains the datastream manager for your middleware.
    - in morse.middleware.your_middleware, a set of files implementing
      different datastream handler (see :doc:`adding_datastream_handler`) for
      different messages. It is recommended to abstract generic handling of
      your middleware in two base class (Publisher and Reader).

The builder part
----------------

To facilitate the use of your middleware, you can add some entries in
:py:mod:`morse.builder.data`. In particular, you need to add an entry for your
datastream manager in :py:data:`morse.builder.data.MORSE_DATASTREAM_MODULE`, and
add the necessary entries for the different datastream handler in
:py:data:`morse.builder.data.MORSE_DATASTREAM_DICT`.
