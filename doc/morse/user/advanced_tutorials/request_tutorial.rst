Services tutorial :tag:`service` :tag:`socket`
==============================================

MORSE has a concept of services: services are remote procedure call commonly
used to configure or control the behaviour of the simulation.

This tutorial shows how to use those services. Different middleware are
supported refer to the :doc:`Middleware support in MORSE <../integration>`
(search requests in the :ref:`compatibility-matrix`).


Pre-requisites
--------------

- You must have completed the :doc:`first tutorial
  <../beginner_tutorials/tutorial>`.


Using the services
------------------

In order to use a service, you have to know the exact name of this service, and
its parameters (You can refer to the component page, available :doc:`here
<../../../components_library>`).

Here is an example using human component (the services available are
documented :doc:`here <../others/human>`).

Scene description
+++++++++++++++++

Let's start to describe the Morse scene. Here, the interesting part is that we
activate the service for human using ``add_service`` method.

.. code-block:: python

	from morse.builder import *

	human = Human()
	human.add_service('socket')
	env = Environment('empty', fastmode=True)

Controlling the human with telnet
+++++++++++++++++++++++++++++++++

Now, let's see how we can control it using telnet::

    $ telnet localhost 4000
	Trying 127.0.0.1...
	Connected to localhost.localdomain.
	Escape character is '^]'.
    > id1 human move [0.1, 0.0]
	id1 SUCCESS

where ``id1`` is the arbitrary id of the request, ``human`` is the name of the
used component, ``move`` and ``[0.1 ,0.0]`` are respectively the name of the
request and its parameters. This parameters have to be in a json list.

Controlling the human with socket :tag:`socket`
+++++++++++++++++++++++++++++++++++++++++++++++

Here is a simple example of using services with sockets in python (available at
``$MORSE_ROOT/tools/simple_exemple.py``):

.. code-block:: python

	import sys
	import socket
	import tty, termios

	HOST = '127.0.0.1'
	PORT = 4000

	def getchar():
		""" Returns a single character from standard input """

		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch


	def _connect_port(port):
		""" Establish the connection with the given MORSE port"""
		sock = None

		for res in socket.getaddrinfo(HOST, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
			af, socktype, proto, canonname, sa = res
			try:
				sock = socket.socket(af, socktype, proto)
			except socket.error:
				sock = None
				continue
			try:
				sock.connect(sa)
			except socket.error:
				sock.close()
				sock = None
				continue
			break

		return sock

	def main():
		sock = _connect_port(PORT)
		if not sock:
			sys.exit(1)

		print("sock connected")
		print("Please press q to quit and use 8456 to move")
		esc = 0
		_id = 0

		while not esc:
			c = getchar()
			speed = 0
			rot = 0
			if (c == "8"):
				speed = 0.1
			elif (c == "5"):
				speed = -0.1
			elif (c == "4"):
				rot = 0.1
			elif (c == "6"):
				rot = -0.1
			if (speed != 0 or rot != 0):
				data_out = "id%d human move [%f, %f]\n" % (_id, speed, rot)
				sent = sock.send(data_out)
				print ("SENT DATA (%d bytes): %s" % (sent, data_out))
				_id = _id + 1

			if c == "q":
				esc = 1

		sock.close()
		print("\nBye bye!")

	main()


.. note::
  You can find a more complete example of python file using services to move
  the human here: ``$MORSE_ROOT/tools/wiimote_human_client.py``. (using wiimote
  to control the human)

Controlling the human with pymorse :tag:`pymorse`
+++++++++++++++++++++++++++++++++++++++++++++++++

The previous example can be rewritten more easily using the :doc:`pymorse API
<../../pymorse>`.

.. code-block:: python

	import sys
	import tty, termios
	from pymorse import Morse

	def getchar():
		""" Returns a single character from standard input """

		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

	def main():
		with Morse()  as  morse:
			print("Please press q to quit and use 8456 to move")
			esc = 0

			while not esc:
				c = getchar()
				speed = 0
				rot = 0
				if (c == "8"):
					speed = 0.1
				elif (c == "5"):
					speed = -0.1
				elif (c == "4"):
					rot = 0.1
				elif (c == "6"):
					rot = -0.1
				if (speed != 0 or rot != 0):
					morse.rpc('human', 'move', speed, rot)

				if c == "q":
					esc = 1

			print("\nBye bye!")

	main()

  
Creating the Service
--------------------

Please, refer to :doc:`Services in MORSE <../../dev/services>`.
