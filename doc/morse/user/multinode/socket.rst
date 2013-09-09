Multi-node Simulation using sockets
===================================


The multinode-server program
----------------------------

Multi-node using sockets requires a central server known as the `multinode-server`.

The MORSE installation includes the server program ``multinode_server``.
By default it will listen on port **6500** for a socket connection from every
one of the simulation nodes. It will receive the updates of the movement of the
robots from all the nodes, condense them in a single list, and send the new
list to all connected clients, thus synchronising the movement of all robots
across the multi-node simulation.


Executing a socket multi-node simulation
----------------------------------------

To start with multi-node simulation, you can try the sample tutorial files located at:
``$MORSE_ROOT/share/morse/examples/tutorials/multinode/``.
You can use these files as a base to create your own scripts.

The files in the tutorials use a slightly more complex setup, since they rely
on a predefined class for the equipped robot.
To run these files, you must go directly into the directory, and change your
``PYTHONPATH`` variable.
Also remember that you must set the ``MORSE_NODE`` variable to identify the nodes.
You can then run MORSE as usual, from that same directory.

The whole process for launching the multi-node simulation is described below:

#. In one shell terminal, launch the multi-node server, with an optional delay::

    $ multinode_server

#. On another terminal, setup the necessary environment variables for ``nodeA`` and launch MORSE:

- On bash::

	$ cd $MORSE_ROOT/share/morse/examples/tutorials/multinode/
	$ export PYTHONPATH=$PYTHONPATH:.
	$ export MORSE_NODE=nodeA
	$ morse tutorial-socket.py

- On csh::

	$ cd $MORSE_ROOT/share/morse/examples/tutorials/multinode/
	$ setenv PYTHONPATH $PYTHONPATH\:.
	$ setenv MORSE_NODE nodeA
	$ morse tutorial-socket.py

#. Do the same for the other node (``nodeB``), either on another terminal or another computer

#. On each MORSE node, start the game engine by pressing :kbd:`p`.

#. On another terminal, connect the middleware ports to instruct the robots to move.
    At the moment, the robots are configured to use YARP, so you need to create the bindings like this::

    $ yarp connect /moveA /morse/robots/ATRV/Motion_Controller/in

#. Give the movement speeds (v, w) to the robot through YARP, and see how it moves in both nodes.

The nodes will be synchronized by the ``multinode_server``: the movements of robots
in one node will be reflected on the other nodes.
