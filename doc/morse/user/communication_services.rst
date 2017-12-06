Communication services
======================

MORSE exposes services which allow us to extract, say, geometric information from
the simulator, which can then be used as the next input to the simulation.
Using these services can show how to query information about the Morse
simulator's internal geometric model.

The services are automatically exposed on a socket interface, by default on port 4000.

All these services belongs to a virtual component called ``communication`` whose
primary purpose is to serve as the foundation of a simple communication model.

Available services
------------------

- ``distance_and_view`` takes two robot names as parameters and returns, if the
  two robots exist, a tuple with the distance between the two robots, and a
  boolean indicating whether there is a clear line of sight between the
  two robots.

Examples
++++++++

Considering the scene 

.. code-block:: python

    from morse.builder import *

    roberta = ATRV()
    robbie = ATRV()
    robbie.translate(x = 10.0)
    env = Environment('indoors-1/boxes', fastmode = True)

you can access the service using telnet::

  > telnet localhost 4000
  Connected to localhost.
  > id1 communication distance_and_view ["roberta", "robbie"]
  id1 SUCCESS [10.004696135303144, true]

Using the pymorse API :tag:`pymorse`, you can access the service in the following
way:

.. code-block:: python

    from pymorse import Morse

    with Morse() as morse:
        res = morse.rpc('communication', 'distance_and_view', 'roberta', 'robbie')
        distance, line_of_site = res
        # distance == 10.004696135303144
        # line_of_site == True
        # ...


.. note::

Communication services are implemented in :py:mod:`morse.services.communication_services`.
