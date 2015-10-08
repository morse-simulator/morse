Human-Robot interaction tutorial
================================

This tutorial shows how to build a simulation with a robot and a human that
is interactively controlled by the simulation user.

In this simple scenario, the robot is ordered to follow the human.


Pre-requisites
--------------

- You must have completed the :doc:`first tutorial <tutorial>`.

Initial scene
-------------

We will use the **Builder API** to create our scene.

Create a new scene ``hri.py`` and add these lines:

.. code-block:: python

    from morse.builder import *

    # Import the human model.
    human = Human()

    # Use the standard environment 'sandox.blend'. You could use any other.
    env = Environment('sandbox')

Launch MORSE with this script (``morse run hri.py``). You can move around the
human with the arrow keys.

.. note:: 
    If you are running MORSE on a Unix machine, you can start your script with
    ``#! /usr/bin/env morseexec``.
    
    Mark then your script as executable (``chmod +x hri.py``). You can now quickly
    start your simulation by calling ``./hri.py``.


Where is my human?
------------------

Exporting the position
++++++++++++++++++++++

As a first step, we would like to export the position of the human in the world.
To do so, we need the :doc:`Pose sensor <../sensors/pose>`.

Appending a pose sensor to the human is easy:

.. code-block:: python

    from morse.builder import *

    human = Human()

    # Import the pose sensor and attach it to the human.
    pose = Pose()
    human.append(pose)

    # [...]

In this tutorial, we will use sockets to stream the pose out of MORSE:

.. code-block:: python

    from morse.builder import *

    human = Human()

    pose = Pose()
    human.append(pose)

    # Set the pose sensor to use the socket interface to communicate 
    # with modules outside of MORSE.
    pose.add_stream('socket')

    # [...]

You can now re-run the simulation, as usual. The human pose is now exported.

Reading the position outside of MORSE
+++++++++++++++++++++++++++++++++++++

We can retrieve the pose of the human from a regular Python script
:tag:`pymorse`:

.. code-block:: python

    from pymorse import Morse

    def printer(data):
        print("Pose=" + str(data))

    with Morse() as morse:

        # The pose sensor is available as 'morse.human.pose' because
        # the human is named 'human' and the pose sensor 'pose' in our
        # Builder script
        morse.human.pose.subscribe(printer)

        # Listen to pose updates for 10 sec
        morse.sleep(10)

You can run this script from any terminal, on the same machine as MORSE (or on
a distant one, just replace ``Morse()`` by ``Morse(<hostname or ip>)``).

It prints on the terminal the pose of the human avatar for 10 seconds. Try to
move the human with the keyboard within the simulator. The output should look
like this::

    Pose={'x': 0.16082972288131714, 'y': 0.00014015310443937778, 'z': 0.047640468925237656, 'pitch': -2.1290716745170357e-08, 'roll': 1.0065883238041806e-08, 'timestamp': 1444319642.4115114, 'yaw': 0.0001225958694703877}
    Pose={'x': 0.16082972288131714, 'y': 0.00014015310443937778, 'z': 0.047640468925237656, 'pitch': -2.1494560797918894e-08, 'roll': 1.0039565623287672e-08, 'timestamp': 1444319642.4276326, 'yaw': 0.0001225958694703877}
    Pose={'x': 0.16082972288131714, 'y': 0.00014015310443937778, 'z': 0.047640468925237656, 'pitch': -2.1901566782389637e-08, 'roll': 1.0047403797841525e-08, 'timestamp': 1444319642.444707, 'yaw': 0.0001225958694703877}
    Pose={'x': 0.16082972288131714, 'y': 0.00014015310443937778, 'z': 0.047640468925237656, 'pitch': -1.7940088525847386e-08, 'roll': 1.0114515447412487e-08, 'timestamp': 1444319642.4619052, 'yaw': 0.0001225958694703877}
    ...

Moving around the human
-----------------------

As you have noticed, you can move the human avatar with the arrow keys. However,
it is also useful to program the motion of the simulated human. Indeed, like any
other robot in MORSE, the human avatar can be externally controlled (for
instance, to perform a predefined trajectory).

Getting the human to follow a path
++++++++++++++++++++++++++++++++++

To get the human to follow a path, we first need to add a :doc:`waypoint actuator<../actuators/waypoint>`, as we did for the pose sensor:

.. code-block:: python

    from morse.builder import *

    human = Human()

    pose = Pose()
    human.append(pose)
    pose.add_stream('socket')

    motion = Waypoint()
    motion.properties(ControlType="Position")
    human.append(motion)
    motion.add_stream('socket')

    env = Environment('sandbox')


You can now re-run the simulation. Using the updated :tag:`pymorse` script
below, you can now send waypoints that the human will follow everytime you press
:kbd:`Enter`.

.. code-block:: python

    from pymorse import Morse

    with Morse() as morse:

        pose = morse.human.pose
        motion = morse.human.motion

        x = 2

        while True:
            input("The human is currently at: %s. Press Enter..." % pose.get())

            x = -x
            y = 0

            print("Moving to %s..." % ([x, y],))
            motion.publish({"x":x, "y":y, 'z':0, 'tolerance':0.3, 'speed':1})


``TUTORIAL WITH THE NEW AVATAR STOPS HERE FOR NOW``

When moving the mouse, you displace the yellow IK target of the head. This
allows you to control the head direction.


Picking objects
---------------

Our human can pick and release objects. Let's add a new object (a cornflakes
box, from the kitchen objects library) on one of the tables. Exit the
simulation (:kbd:`Esc`), and re-open your script.

Add the following lines:

.. code-block:: python

    from morse.builder import *

    human = Human()

    # Import, configure and place a static object from 'kitchen_objects.blend'.
    cornflakes = PassiveObject("props/kitchen_objects", "Cornflakes")
    cornflakes.setgraspable()
    cornflakes.properties(Label = "My cornflakes")
    cornflakes.translate(-7, 3, 1.1)

    env = Environment('indoors-1/indoor-1')

You can learn more on :doc:`passive objects here <../others/passive_objects>`.

.. image:: ../../../media/hri_cornflakes.jpg 
  :align: center

Start again the simulation (``morse run hri.py``), and press the :kbd:`x` key
to switch to the **manipulation mode**. You can control the hand with the mouse
while holding :kbd:`Middle Mouse Button`. Press the :kbd:`Left Mouse Button`
with the crosshairs over an object to pick it, and press :kbd:`Right Mouse
Button` to drop the object.

.. image:: ../../../media/hri_cornflakes_pickup.jpg 
  :align: center

Check the :doc:`human component <../others/human>` documentation for more details on what can be done
with the human component.




