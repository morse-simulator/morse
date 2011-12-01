Multi-node simulation
=====================

MORSE offers the possibility to deploy the simulation on a distributed, 
multi-node infrastructure.

General principle
-----------------

The multi-node infrastructure consists of a set of nodes, that connect with a
server program to synchronise events happening at every node.
In this infrastructure, each node is a MORSE node, i.e. a Blender application
that simulates a MORSE scene as classically done in a mono-node context.

Each node then manages a sub-set of the robots present in the simulated world.
"Managing a robot" means that the node will export the data of the robot's sensors,
and apply the commands of the robot's controller, from classical MORSE middlewares.

Every node will load a very similar scenes to all other nodes,
but with very specific differences to indicate what robots will be handled by each
node and how the nodes will synchronise.
Currently the configuration of the different nodes is done through the 
:doc:`Builder API <../../../dev/builder>`.

Each node considers the robots it is not managing as *External*:
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
:doc:`Builder API <../../../dev/builder>`.
Initially, the scene is generated the same way as in a single node simulation.
Additionally, it is necessary to indicate which robots will be externally managed
and also provide the information about the central server that will provide the
synchronization.

Multi-node options in Builder API
+++++++++++++++++++++++++++++++++

In general, the builder scripts for every node must be almost the same, except for
a couple of special functions that will differentiate how each node operates.
Any scenario created with the Builder API will have an instance to the ``Environment``
class and one or more instances of the ``Robot`` class. Objects of these classes have
to be properly configured for multi-node.

The ``Robot`` class in Builder has a method to indicate when a robot is external,
for example::

    robo1 = Robot('atrv')
    robo1.make_external()

The ``Environment`` object must also be configured with the information about the
server that will coordinate the nodes::

    env = Environment('land-1/trees')
    env.configure_node(protocol="socket", node_name="NODE A", server_address="140.93.0.93", server_port="65000")

The parameters to the `configure_node` method are the following:
  - **protocol**: currently limited to two options: 'socket' or 'hla' (see list of protocols below)
  - **node_name**: unique name for every node, used to identify them
  - **server_address**: IP address where the synchronisation server is running
  - **server_port**: (optional) used for the socket protocol. It should always be 65000

Supported protocols
-------------------

.. toctree::

    user/multinode/socket
    user/multinode/hla
