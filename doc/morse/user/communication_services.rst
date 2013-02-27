Communication services
======================

MORSE exposes services which allow to extract geometric information from
the simulator which can be next used as input for communication simulation. It
can be seen as a simple example about how to query information about the inner
geometric model of Morse simulator.

The services is automatically exposed on a socket interface, on port 4000.

All these services belongs to a virtual component called ``communication`` as
its primary purpose is to serve as a base for a simple communication model.

Available services
------------------

- ``distance_and_view`` takes two robot names in parameter and returns, if the
  two robots exists, a tuple with the distance between the two robots, and a
  boolean representing the direct visibility between these two robots.

Examples
++++++++

Considering the scene 

.. code-block:: python

    from morse.builder import *
    mana = ATRV()
    minnie = ATRV()
    minnie.translate(x = 10.0)
    env = Environment('indoors-1/boxes', fastmode = True)

you can access to the service using telnet::

  > telnet localhost 4000
  Connected to localhost.
  > id1 communication distance_and_view ["mana", "minnie"]
  id1 SUCCESS [10.004696135303144, true]

Using pymorse API :tag:`pymorse`, you can access the service in the following
way:

.. code-block:: python

    from pymorse import Morse

    with Morse() as morse:
        res = morse.rpc('communication', 'distance_and_view', 'mana', 'minnie')
        # res[0] = 10.004696135303144
        # res[1] = True
        # ...


.. note::

  Communication services are implemented in :py:mod:`morse.services.communication_services`.
