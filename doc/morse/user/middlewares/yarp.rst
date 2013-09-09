YARP
====

The YARP middleware provides a simple method to share data across components in
different computers. It is based on the concept of *ports*, which are
registered in a *name server* to differentiate between separate channels of
communication.

Data is shared in the format of bottles, which are nested data structures of
various data types.

.. note:: Port names used in MORSE have the following format:
  ``/morse/component_name/[direction]``. The term [direction]
  is either ``in`` or ``out``, depending on the type of component being actuator
  or sensor, respectively.

Files
-----

- Python: ``$MORSE_ROOT/src/morse/middleware/yarp_datastream.py``

.. _yarp_ds_configuration:

Configuration specificities
---------------------------

When configuring a component to export its data through YARP, you can pass
several options to tune the behaviour of the {ex, im}porter.

The option ``port`` allows to select the explicit name of the port exported by
YARP, instead of the default name. If existing, the option``topic`` will tell
the component to connect automatically to the named topic.

.. code-block :: python

    foo.add_stream('yarp', port = '/foo/bar', topic = '/this/interesting/topic')


Service interface
-----------------

MORSE components providing :doc:`services <../../dev/services>` can expose this interface using YARP ports.

.. note:: Services handled with YARP will create two ports, named:
  ``/ors/services/[component_name]/[service_name]/request`` and
  ``/ors/services/[component_name]/[service_name]/reply``.
  Where [service_name] is a string consisting of ``[component_name]#[service_method]``.

The (ASCII) protocol of the requests is simple::

  id component service "[parameters]"

- ``id`` is a freely chosen request id. It is mainly useful to identify answers
  from asynchronous services.  
- ``component`` is the name of the component you want to invoke the service on.

.. note::
  Services that control the whole simulator belong to the special component ``simulation``.

- ``service``: the name of the request to invoke.
- ``parameters`` (can be omitted if the request takes no argument): request
  arguments. Arguments must be enclosed in brackets, and surrounded by quotes.

MORSE answer follows this model::

  id status [result]

- ``id`` the same id the client used to send the request,
- ``status``: one of the :py:mod:`morse.core.status` constants
- ``result``: a JSON-like result (actually, the representation of the Python
  result object), if any.

Example (sending request on terminal 1)::

  $ yarp write /request /ors/services/Motion_Controller/Motion_Controller#goto/request
  yarp: Port /request active at tcp://140.93.0.93:10173
  yarp: Sending output from /request to /ors/services/Motion_Controller/Motion_Controller#goto/request using tcp
  req1 Motion_Controller goto "[5.0, 4.0, 0.0]"

Example (receiving reply on terminal 2)::

  $ yarp read /reply /ors/services/Motion_Controller/Motion_Controller#goto/reply
  yarp: Port /reply listening at tcp://140.93.0.93:10174
  yarp: Receiving input from /ors/services/Motion_Controller/Motion_Controller#goto/reply to /reply using tcp
  "0 SUCCESS Stop"


The YARP service interface is implemented in :py:mod:`morse.middleware.yarp_request_manager`.



Known problems
--------------

Some blender files will start the simulation fine the first time after opening the
file, but stopping the simulation and starting it again will give this error::

    def open(self, *args): return _yarp.Contactable_open(self, *args)
    NotImplementedError: Wrong number of arguments for overloaded function 'Contactable_open'.

To correct this, it is necessary to open the Blender file, then on a Text
Editor window, select the file ``load_yarp.py``. To the right of the file name,
there is a checkbox named ``Register``. Check this checkbox, save the file, and
open it again. This will ensure that the script ``load_yarp.py`` is read every
time the file is opened, so that yarp is correctly setup before launching the simulation.

