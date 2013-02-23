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
<../../../components_library>`

Here is an example using human component (the services available are :doc:`here
<../others/human>`) with telnet::

    $ telnet 127.0.0.1 4000
    $ id1 Human move (0.1,0.0)

where ``id1`` is the arbitrary id of the request, ``Human`` is the name of the
used component, ``move`` and ``(0.1,0.1)`` are respectively the name of the
request and its parameters. This parameters have to be in a tuple ( () or [] ).

Here is a simple example of using services with sockets in python (available at
``$MORSE_ROOT/tools/simlple_exemple.py``):

.. code-block:: python

    import sys
    import time
    import socket
    import tty, termios

    id_ = 0

    HOST = '127.0.0.1'
    PORT = 4000 
    s = None

    def getchar():
       #Returns a single character from standard input
       
       fd = sys.stdin.fileno()
       old_settings = termios.tcgetattr(fd)
       try:
          tty.setraw(sys.stdin.fileno())
          ch = sys.stdin.read(1)
       finally:
          termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
       return ch

    def main():
        global s
        global id_
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        print "Socket connected"
        print "Please press q to quit and use 8456 to move the human"
        esc = 0
        while not esc :
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
            if (speed != 0 or rot != 0) :
                msg = "id" + str(id_) + " Human move (" 
                msg += str(speed) + "," 
                msg += str(rot) + ")\n"
                s.send(msg)
                id_ = id_ + 1
            if c == "q" :
                esc = 1
        s.close()
        print "\nBye bye!"

    main()


.. note::
  You can find a more complete example of python file using services to move
  the human here: ``$MORSE_ROOT/tools/wiimote_human_client.py``. (using wiimote
  to control the human)
  
  
Creating the Service
--------------------

Please, refer to :doc:`Services in MORSE <../../dev/services>`.
