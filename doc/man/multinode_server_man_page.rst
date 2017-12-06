:orphan:

multinode_server manual page
=================================

Synopsis
--------

**multinode_server** [synchronisation delay]


Description
-----------

This is a socket server program to synchronise several instances of MORSE (nodes)
running the same simulation scenario.
This program must be started before launching the simulation in the nodes.

All the client nodes will connect with the server and send it the updated
positions of the robots they manage. The **multinode_server** program will
accumulate the positions of all the robots into a single Python dictionary,
and then send this back to all the clients.

The server also supports pausing the simulation, by typing in its terminal **p**
and **Enter**. The same command will allow the simulation to continue.
This mechanism relies on the fact that the clients will remain waiting for
a reply from the server before continuing with the simulation.


Parameters
----------
:[synchronisation delay]:
        Optional parameter that defines the delay (in seconds) between messages
        to the connected clients. A larger delay will slow down the simulation
        in all the clients. The default value is 0.0.

See Also
--------
:manpage:`morse(1)` :manpage:`morseexec(1)`
