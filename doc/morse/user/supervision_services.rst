Supervision services
====================

MORSE exposes several services to remotely manage and monitor the simulator.

These services are automatically exposed on a socket interface, on port 70000.

All these services belongs the a virtual component called ``simulation`` and
follows the normal syntax for socket requests.  Thus, a request to retrieve the
list of robots would look like that::

  > telnet localhost 70000
  Connected to localhost.
  > req1 simulation list_robots
  req1 OK ['Robot1', 'Robot2']


Available services
------------------

- ``list_robots`` (no parameter): returns the list of the robots currently available in the simulation

.. note::
  Simulation services are stored in :py:mod:`morse.core.supervision_services`.
