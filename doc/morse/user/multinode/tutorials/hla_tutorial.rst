HLA-based multi-node simulation
===============================

In this tutorial you can experiment distributing your MORSE simulation using HLA.
You'll be able to connect two simulation nodes, each one controlling one robot.

Setup
-----

You need to install the HLA libraries for MORSE, by following the :doc:`HLA documentation <../hla>`.

For this tutorial, we also use YARP to control the robot. 
It could be a good starting point to first follow the :doc:`YARP specific tutorial <../../beginner_tutorials/yarp_tutorial>`.

Before running a simulation using YARP, it is necessary to open a new shell terminal and start the ``yarpserver3`` program::

  $ yarpserver3

To setup the HLA federation, you also need the RTI gateway to be running. Open a new shell terminal and start it::

  $ rtig

Configuring the scenario
------------------------

You can find the tutorial file in ``$MORSE_ROOT/share/morse/examples/tutorials/multinode/tutorial-hla.py``.

The file itself is very simple, just defiing two robots in an indoor enrironment.
The multinode configuration is done in lines:

.. code-block:: python

   env.configure_multinode(protocol="hla", server_port=60400, 
       distribution={
           "nodeA": [dala1.name],
           "nodeB": [dala2.name],
       })

It uses the 'hla' protocol, connects to the RTIg using port 60400 (default for CERTI), and distributes
the simulation by attaching robot dala1 to nodeA, and robot dala2 to nodeB.
If the RTIg is launched on another host, you can set the server_address to the host IP. 

Lauching the simulation
-----------------------

To start each simulation node, just launch MORSE with the node name option and the tutorial script::

  $ morse run --name nodeA tutorial-hla.py
  
Idem for nodeB.

The two nodes are now HLA-synchronized!

Controlling the simulation
--------------------------

Just launch a YARP terminal that connects to one of the robot controller::

  $ yarp write /controlA /morse/robots/ATRV/Motion_Controller/in

You can then enter (v, w) command in this terminal (for instance by sending ".2 .2").
At this moment, you control robot dala1 on nodeA, and as the nodes are synchronized, dala1 pose is reflected in nodeB! 

You can do the same thing for the other node with::

  $ yarp write /controlB /morse/robots/ATRV.001/Motion_Controller.001/in
