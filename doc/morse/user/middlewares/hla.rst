HLA
===

HLA, as a middleware, provides a way to connect Morse to various other
simulator engines, using HLA as standard way to communicate between theses
tools. As Python interface is not defined for HLA, the Morse implementation
works only with the CERTI implementation of HLA. Moreover, currently, Morse
deals only with a subset of the full HLA standard:

- basic federation management
- declaration management (attribute part only, i.e. datastream in the Morse
  language)
- object management (note that, in the current state, Morse does not create
  new object when it receives a ``Discover Object Instance`` but links a
  pre-existing object with this HLA instance
- time management

In particular, it means that in the current state, Morse does not support:

- interactions
- data distribution management 
- ownership management

.. warning::

    In the current state, no SOM (Simulation Object Model) has been specified,
    so it is up to the specific users to develop it, with the associated Morse
    bindings.
    
    ``certi_test_input`` and ``certi_test_output`` are provided for example,
    and are supposed to work with the billard example provided by CERTI.

Datastreams interface
---------------------

The mapping between Morse and HLA vocab is the following:

- ``publishObjectClass`` and ``UpdateAttributeValues`` correspond to an output
  stream
- ``subscribeObjectClassAttributes`` and ``reflectAttributeValues``
  correspond to an input stream.

Service interface
-----------------

It is unsupported for the moment.

Configuration specificities
---------------------------

The following variables permit to configure HLA behaviour:

- **fom**: Mandatory : a string representing the FOM (Federation Object Model) used for
  this simulation
- **name**: Mandatory : a string representing the name of this simulation in
  the federation
- **federation**: Mandatory : the name of the federation to join
- **sync_point**: Optional : the name of the initial synchronisation point
  used for the federation. If it is not present, Morse assumes there is no
  need to initial synchronisation
- **sync_register**: Optional : if a **sync_point** is defined, and this
  variable is true, then this instance of Morse will register the
  **sync_point** to the RTIG, and the user should start the simulation by
  pressing :kbd:`Enter`. The default is False.
- **time_sync**: Optional : a boolean indicating if the simulation is in
  'Best-effort' mode or synchronised by HLA. The default value is False.
- **timestep**: Optional : a float indicating how much logic time elapse each
  game loop. **Do not set it if you use physics from Blender (or sensors
  relying on Blender time !**.
- **lookahead** Optional : a flow indicating the lookahead used by HLA. The
  default value is **timestep**.
- **stop_time**: Optional : a float indicating when the simulation must stop.
  If not configured, the simulation continues infinitely. It only works if
  **time_sync** is enabled.


You need to pass them to the ``configure_stream_manager`` method in the
following way:

.. code-block :: python


    env = Environment('...')
    env.configure_stream_manager(
            'hla', 
             fom = 'Test.fed', name = 'Morse', federation = 'Test',
             sync_point = 'Init', time_sync = True)

Files
-----

- Python: ``$MORSE_ROOT/src/morse/middleware/hla_datastream.py``

