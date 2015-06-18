MORSE quickstart
================

Creating your first simulation with MORSE is easy.

First, the boring part:

- :doc:`Install MORSE<user/installation>`
- Open a terminal and check everything is well configured

::

 $ morse check

Then, the funny part::

 $ morse create mysim
 $ morse run mysim

This should be enough to launch MORSE and display a sample simulation. You can
already control the robot with the arrow keys and check collisions with
surrounding objects. Press :kbd:`h` to display the list of keyboard shortcuts.

.. image:: ../media/environments/sandbox.jpg
  :align: center

So, what happened here?

`morse create mysim` actually created a new subfolder in the current directory.

If you inspect it, you will find a Python file called `default.py` and two
folders. `default.py` is what we call a **Builder script** (because it uses MORSE's :doc:`Builder API<user/builder>`) and it describes your simulation.

.. note::
  Note that the script is still Python: besides the Builder API, you can use any
  standard Python constructs to program your simulation.

Open `default.py` in your favorite text editor. There are plenty of comments and
it should be mostly self-explanatory: it builds a simulation with one robot,
called *Morsy*, two actuators (including the keyboard for debugging/testing),
one sensor, and these simulated components talk to the outside world via a
socket.

By checking the :doc:`component library<components_library>`, you get a first
glimpse on what standard robots, actuators and sensors are available *out of
the box*. You can add new elements to your simulation by directly editing
`default.py`.

.. note::
  All simulations in MORSE are (by default) self-contained. This means that you
  can share your simulation folder (zipping it, sharing with GIT, etc.) with
  others, they can import it with `morse import <path>` (only required the
  first time), and then run it exactly as you did. Convenient to work on a shared
  project.

What if you want to create a custom robot or a custom actuator/sensor?

::

 $ morse add robot MyCustomRobot mysim

This will create a template for your robot. Open the robot description as
hinted on the output. You will see that by default, your custom robot has the
same two actuators and sensor as in `default.py`. You can change them to build
a robot that match your need.

Check as well `morse add actuator` and `morse add sensor` if you need to create
custom actuators/sensors.

That's all folks! You now know the basics of MORSE. Head to the
:doc:`tutorials` section to learn how to interact with your simulated robots
and for more advanced examples.



