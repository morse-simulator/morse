Multi-node Simulation using sockets
===================================


Configuring the socket environment
----------------------------------

Multi-node using sockets requires a central server known as the `multinode-server`.

There is a sample server program written in Python at 
``$MORSE_ROOT/examples/morse/clients/multinode/multinode_server.py``
By default it will listen on port **6500** for a socket connection from every one of
the simulation nodes.

Executing an socket multi-node simulation
-----------------------------------------

Once you have generated your node scenarios, you are ready to launch the 
multi-node simulation.

1. Launch the multi-node server::

    $ python $MORSE_ROOT/examples/morse/clients/multinode/multinode_server.py

2. On each node, launch the correct MORSE scenario, using::

    $ morse exec <scenario_node.py>

3. Then, on each node, launch the game engine by pressing :kbd:`p`.

The nodes are now synchronized: move one robot on the node where it is
managed, its pose will be reflected on the other nodes.

To start with multi-node simulation, you can try the sample tutorial files located at:
`$MORSE_ROOT/examples/tutorials/multinode/`
