Multi-node simulation
=====================

MORSE offers the possibility to deploy the simulation on a distributed, 
multi-node infrastructure.
The advantage of using several MORSE nodes is to permit simulation of large
numbers of robots, without slowing down a single instance of MORSE.

General principle
-----------------

The multi-node infrastructure consists of a server program whose task is to
synchronise the events happening in a series of simulation nodes.
The simulation nodes as normal instances of MORSE (*i.e.* the Blender application
running a simulation scene), all of them connected with the multi-node server,
and running the same scene.

Each node manages a sub-set of the robots present in the simulated world.
"Managing a robot" means that the node will export the data of the robot's sensors,
and apply the commands of the robot's controller, from classical MORSE middlewares.

Every node will load the same simulation scene as all other nodes,
but with a different configuration to indicate what robots will be handled by each
node and how the nodes will synchronise.
Currently the configuration of the different nodes is done through the 
:doc:`Builder API <../../../user/builder>`.

Each node considers the robots it does not manage as *External*:
an external robot is not updated by the node itself (robot's components are disabled),
but the position and orientation of the robot are updated across all nodes in order
to reflect the movements of the robot in the node charged of managing it.

The multi-node infrastructure then ensures the consistency of all the simulated nodes:
at each time, all the robots' poses are equivalent in every node. Moreover,
it decreases the processing load of each node, that only has to process the
sensor/command of the managed robots.

Setting up the simulation scenes
--------------------------------

The definition of multi-node scenarii is done by means of the
:doc:`Builder API <../../../user/builder>`.
Initially, the scene is generated the same way as in a single node simulation.
Additionally, it is necessary to configure which robots will be managed by each
node, provide the information about the central server that will do the
synchronization, and finally to differentiate each node.

Multi-node options in Builder API
+++++++++++++++++++++++++++++++++

In general, all the nodes used in a multi-node simulation must use the same
builder script to generate the simulation scenario.
However, the builder script must include the information about the robots that
will be handled by each node.
This is done through the instance of the ``Environment`` class that must be in
any builder script. The method ``configure_multinode`` must be called to configure
the information about the server that will coordinate the nodes.
In this example we show part of a builder script for a multi-node scene:

.. code-block:: python

    dala1 = Robot('atrv')
    dala2 = Robot('atrv')

    env = Environment('land-1/trees')
    env.configure_multinode(    protocol='socket',
                                server_address='localhost',
                                server_port='65000',
                                distribution={
                                    "nodeA": [dala1.name],
                                    "nodeB": [dala2.name],
                                })

The parameters to the `configure_multinode` method are the following:
  - **protocol**: currently limited to two options: 'socket' or 'hla' (see list of protocols below)
  - **server_address**: (optional) IP address where the synchronisation server is running. The default value is 'localhost'
  - **server_port**: (optional) used for the socket protocol. It should always be 65000
  - **distribution**: A Python dictionary. The keys are the names of the nodes, and the values are lists with the names of the robots handled by each node


Differentiating the nodes
+++++++++++++++++++++++++

Before launching MORSE with the properly configured builder script,
it is necessary to indicate the node name that will be used by the
instance of MORSE. This is done by defining the environment variable
``MORSE_NODE`` in the terminal where MORSE will be launched.
The syntax for this varies depending on the shell you are using.
For bash::

  $ export MORSE_NODE=nodeA
  $ morse multinode_scene.py

For csh::

  $ setenv MORSE_NODE nodeA
  $ morse multinode_scene.py

This will allow the Builder API to identify the corresponding node,
and configure the scene correspondingly. In this example, when creating the
scene only the robots configured for 'nodeA' will be initialized, and all
others will be marked as *External* robots.


Supported protocols
-------------------

.. toctree::

    user/multinode/socket
    user/multinode/hla
