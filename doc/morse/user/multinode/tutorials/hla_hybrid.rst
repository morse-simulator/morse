HLA-based hybrid simulation
===========================

In this tutorial, we learn how to use HLA to setup a hybrid simulation. A hybrid 
simulation in the MORSE context is made of at least one MORSE node, connected
in a HLA federation to other nodes, being either other simulators (e.g., think about
a simulation model for a quadrotor available as a standalone library/binary) or
even real robots.

Prerequisites
-------------

You need to install the HLA libraries for MORSE, by following the :doc:`HLA documentation <../hla>`.
We also recommend you to have a look at the :doc:`HLA multi-node tutorial <./hla_tutorial>`.

To setup the HLA federation, you also need the RTI gateway to be running. Open a new shell terminal and start it::

  $ rtig

The MORSE scenario
------------------

You can find the scenario file 
in ``$MORSE_ROOT/share/morse/examples/tutorials/multinode/tutorial-hla-hybrid.py``.

This file is very close to the multi-node scenario. It is made of one robot located 
in the "grande-salle" scene:

.. code-block:: python

	from morse.builder import *
	atrv = ATRV()
	env = Environment('laas/grande_salle')
	env.show_framerate(True)
	env.show_physics(False)

We configure HLA and indicate that the ATRV robot is managed by node "atrv", and we create
the environment:

.. code-block:: python

	env.configure_multinode(protocol="hla", server_address="127.0.0.1", server_port=60400, 
    	distribution={"atrv": [atrv.name],})

	env.aim_camera([1.3300, 0, 0.7854])
	env.place_camera([10.0, -10.0, 3.0])
	env.create()


.. note::

	The node name in distribution is only relevant to indicate to MORSE
	if it has to export the robot position or not. This name will not
	be used by non-MORSE federates.
	
To launch the MORSE node, just launch MORSE with the tutorial scenarion::

	$ morse run ${MORSE_ROOT}/share/morse/examples/tutorials/multinode/tutorial-hla-hybrid.py

Publishing the ATRV position using Python
-----------------------------------------

You can find the example file in ``$MORSE_ROOT/share/morse/clients/hla/hybrid-client.py``.

This sample code is dedicated to be integrated in a non-MORSE federate (another simulator
or a real robot architecture) in order to publish the robot position so that
it is correctly reflected into MORSE. Let's look at some relevant parts of the code.

.. code-block:: python

	import hla
	import hla.rti as rti
	import hla.omt as fom
	
These lines import the HLA python modules. They are installed by the pyHLA library.

.. code-block:: python

	MorseVector = fom.HLAfixedArray("MorseVector", fom.HLAfloat32LE, 3)
	
This line define the vector type used for MORSE robots position and orientation.

.. code-block:: python

	class MorseHLAClient():
		def __init__(self, robot_name, host="localhost", port=60400):
			self.fom = "morse.fed"
			self.federation = "MORSE"
			self.robot = None
			if os.getenv("CERTI_HTTP_PROXY") == None:
				os.environ["CERTI_HTTP_PROXY"] = ""
			os.environ["CERTI_HOST"] = str(host)
			os.environ["CERTI_TCP_PORT"] = str(port)

The MorseHLAClient is the main class of the example. The first lines of its constructor
define some useful variables. The fom and federation name must not be changed: they are defined
in the HLA plugin of MORSE. The CERTI environment variables are used to locate where
the rtig has been launched on the network.
        
.. code-block:: python

		self.rtia = rti.RTIAmbassador()
		self.rtia.createFederationExecution(self.federation, self.fom)
		self.morse_ambassador = rti.FederateAmbassador()
		self.rtia.joinFederationExecution("hla-client", self.federation, self.morse_ambassador)

The constructor continues by creating the RTI Ambassador and a Federate Ambassador. It creates
the federation and joins it.

.. code-block:: python

		self.robot_t = self.rtia.getObjectClassHandle("Robot")
		self.position_t = self.rtia.getAttributeHandle("position", self.robot_t)
		self.orientation_t = self.rtia.getAttributeHandle("orientation", self.robot_t)
		self.rtia.publishObjectClass(self.robot_t, [self.position_t, self.orientation_t])
		self.robot = self.rtia.registerObjectInstance(self.robot_t, robot_name)

Then, it gets from the RTIG some handlers on the data types that will be manipulated,
declares that is will publish a robot object with attributes position and orientation,
and finally register the "ATRV" robot.

.. code-block:: python

		def send(self, x, y):
			hla_att = {self.position_t: MorseVector.pack([x, y, 0]),
					self.orientation_t: MorseVector.pack([0, 0, 0])}
			self.rtia.updateAttributeValues(self.robot, hla_att, "update")
			self.rtia.tick()
        
The send function sends the robot position to the HLA federation.

You can send this client with::

	$ python3 $MORSE_ROOT/share/morse/clients/hla/hybrid-client.py
	
The ATRV position will randomly change in MORSE according to the data sent by the client.
Congratulations, you have performed some hybrid simulation!

Subscribing to the ATRV position using Python
---------------------------------------------

When performing hybrid simulation, you may be interested in getting the ATRV
position from MORSE to integrate in your specific simulation federate (think
about a communication simulator that needs the robot position to simulate the
communication quality without using the Blender physics).

Subscribing to an object in HLA is based on callbacks, called when receiving
messages (existence of new object, new data published, ...) The python code is hence
a bit less strait-forward than for publishing.

If you are interesting in such a behavior, look at the HLA plugin for MORSE, that
actually implements publishing/subscribing behaviors. You can find it in
``$MORSE_ROOT/lib/python3.2/site-packages/morse/multinode/hla.py``.


