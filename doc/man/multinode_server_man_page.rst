:orphan:

multinode_server manual page
=================================

Synopsis
--------

**multinode_server** [synchronisation delay]


Description
-----------

Socket server program to synchronise several instances of MORSE (nodes)
running the same simulation scenario.
This program must be started before launching the simulation in the nodes.

All the client nodes will connect with the server and send it the updated
positions of the robots they manage. The **multinode_server** program will
concentrate the positions of all robots into a single Python dictionary,
and then send it back to all clients.

It also permits pausing the simulation, by typing in its terminal **p**
and **Enter**. The same command will allow the simulation to continue.
This mechanism relies on the fact that the clients will remain waiting for
a reply from the server before continuing with the simulation.


Parameters
----------
:[synchronisation delay]:
        Optional parameter that defines the delay (in seconds) between messages
        to the connected clients. A larger delay will slow down the simulation
        in all clients. Default value is 0.0

See Also
--------
:manpage:`morse(1)` :manpage:`morseexec(1)`
