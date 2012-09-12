Cat and mouse game tutorial
===========================

This tutorial will provide a more complex scenario, with a clear objective to
accomplish for a robot, and that will show how to "close the loop" of sensors
and actuators simulated in MORSE.

We will make a chase game, where a robot controlled by a human (the mouse) will
be chased by a second robot (the cat) that is running software external to
MORSE.  This will be accomplished by the use of sensor and actuators configured
properly.

.. image:: ../../../media/MORSE-cat_mouse.png
   :align: center

Pre-requisites
--------------

- You must have completed the :doc:`first tutorial <../beginner_tutorials/tutorial>`.

Creating the scenario
---------------------

We'll use the Builder API to configure the robots in the scenario.
First we will configure the *Mouse* robot, which is a lot simpler.

- Create a new ATRV robot, and change its name to ``MOUSE``:

  .. code-block:: python

    from morse.builder import *

    Mouse = Robot('atrv')
    Mouse.name = "MOUSE"
    Mouse.translate (x=1.0, z=0.2)

- We will give it some special properties, so that it can be recognised by the
  semantic camera sensor:

  .. code-block:: python

    Mouse.properties(Object = True, Graspable = False, Label = "MOUSE")

- Next we make it controllable by the keyboard, using the correct actuator.
  Also, we change the default speed, to make it more agile

  .. code-block:: python

    Keyb = Actuator('keyboard')
    Keyb.properties(Speed=3.0)
    Mouse.append(Keyb)


Now we'll create the *Cat* robot, with a sensor to detect the mouse, and an
actuator to follow it.

- Create another ATRV robot and set its name to ``CAT``:

  .. code-block:: python

    Cat = Robot('atrv')
    Cat.name = "CAT"
    Cat.translate(x=-6.0, z=0.2)

- Next add two :doc:`semantic cameras <../sensors/semantic_camera>` to the
  robot. This will provide us with an easy way to follow our target.
  One is placed right and one left, to provide a fake stereo vision:

  .. code-block:: python

    Semantic_L = Sensor('semantic_camera')
    Semantic_L.translate(x=0.2, y=0.3, z=0.9)
    Semantic_L.name = 'Camera_L'
    Cat.append(Semantic_L)

    Semantic_R = Sensor('semantic_camera')
    Semantic_R.translate(x=0.2, y=-0.3, z=0.9)
    Semantic_R.name = 'Camera_R'
    Cat.append(Semantic_R)

- Add also a :doc:`v, omega actuator <../actuators/v_omega>` that will make
  the robot move:

  .. code-block:: python

    V_W = Actuator('v_omega')
    Cat.append(V_W)

- We configure these two components to use the :doc:`sockets middleware <../middlewares/socket>`:

  .. code-block:: python

    V_W.configure_mw('socket')
    Semantic_L.configure_mw('socket')
    Semantic_R.configure_mw('socket')

And finally we complete the scene configuration:

  .. code-block:: python

    env = Environment('land-1/trees')
    env.place_camera([10.0, -10.0, 10.0])
    env.aim_camera([1.0470, 0, 0.7854])
    env.select_display_camera(Semantic_L)

The last line indicates to MORSE that you want the images seen from the left
camera to be displayed on the HUD screen, visible when you press :kbd:`v`
during the simulation.
You can easily change it to display the view of the right camera.

The complete script can be found at: ``$MORSE_SRC/examples/tutorials/cat_mouse_game.py``.


Testing the output
------------------

You can check that the data from the cameras is being correctly streamed, by
launching the simulator and connecting to the data ports via telnet.

Run morse with the builder script to create the scenario::

  $ cd MORSE_SRC/examples/tutorials
  $ morse run cat_mouse_game.py

Then start the simulation pressing :kbd:`p` in Blender. On the terminal you
will get messages indicating the port numbers used by the semantic cameras.
Normally they should be:

  - Right camera: ``60001``
  - Left camera: ``60002``

Try connecting to these ports using the ``telnet`` program on another terminal,
and you should see the data of object visibility comming from the cameras::

  $ telnet localhost 60001


Control program
---------------

As a very simple example of how to use the data from a sensor to drive the
robot, we'll create a Python script to connect to MORSE and provide the
"reasoning" of the ``CAT`` robot.

The whole program can be found at: ``$MORSE_SRC/examples/clients/atrv/cat_script.py``
Here we'll explain the main parts of it:

- The function ``is_mouse_visible`` will use the specified semantic camera to
  check if the ``MOUSE`` robot is anywhere in front:

  .. code-block:: python

    def is_mouse_visible(side):
        """ Read data from the semantic camera, and determine if a specific
        object is within the field of view of the robot """
        socket_name = "semantic_%s" % side
        semantic_data = _read_socket_message(socket_name)
        if semantic_data:
            for item in semantic_data:
                if item['name'] == "MOUSE":
                    return True
        return False


- The main decision to move is made based on the information from the
  semantic cameras.
  There are four cases possible: The mouse can be seen by both cameras at
  once, only by the right, only by the left or by none of them.
  The ``CAT``'s logic is very simple, it will move forward when the ``MOUSE``
  is seen by both cameras, turn to the side of the only camera that sees the
  target or turn in place until it sees the target ``MOUSE``.

  .. code-block:: python

    def chase_mouse():
        """ Use the semantic cameras to locate the target and follow it """
        mouse_seen_left = False
        mouse_seen_right = False

        while True:
            mouse_seen_left = is_mouse_visible("L")
            mouse_seen_right = is_mouse_visible("R")
            if mouse_seen_left and mouse_seen_right:
                v_w = {"v": 2, "w": 0}
            elif mouse_seen_left:
                v_w = {"v": 1.5, "w": 1}
            elif mouse_seen_right:
                v_w = {"v": 1.5, "w": -1}
            else:
                v_w = {"v": 0, "w": -1}

            data_out = (json.dumps((v_w)) + '\n').encode()
            sent = sockets['motion'].send(data_out)

- The client script can be run from a terminal with the command::

  $ python3 cat_script.py [motion_controller_port_number] [left_camera_port_number] [right_camera_port_number]

- The optional parameters for the port numbers are used only if MORSE opens
  the ports at different addresses from the ones expected by the program,
  which are:

  - Motion_Controller: ``60000``
  - Right camera: ``60001``
  - Left camera: ``60002``


Running the game
----------------

Run morse with the builder script to create the scenario. Then start the
simulation pressing :kbd:`p` in Blender. You will be able to control the
``MOUSE`` robot with the arrow keys on the keyboard.

Run the Python control script from a terminal. The ``CAT`` mouse will start
moving and using the data from its cameras to chase after the ``MOUSE``.


Going further
-------------

This example is very basic, but already provides a test of how the use of
sensor data can help drive a robot.  You can substitute the simple Python
client that controls the ``CAT`` for a more complex piece of software,
implemented in other languages and middlewares.  Here are some ideas of what
you could do to improve the "intelligence" of the ``CAT``.

- Use a single semantic camera and a :doc:`Pose sensor <../sensors/pose>` to
  follow the mouse. You don't really need two semantic cameras, since among the
  data one provides is the location of the detected object. Using that and the
  current position of the ``CAT``, it will be possible to chase, but you need
  to do some calculations to determine in which direction to turn

- Use a :doc:`Sick sensor <../sensors/sick>` to make the ``CAT`` detect and
  avoid obstacles. This is more complex, since you have to handle a lot of data
  that is streamed by the Sick

- The target could hide behind an obstacle, so you could implement a strategy
  to move around the area searching for it
