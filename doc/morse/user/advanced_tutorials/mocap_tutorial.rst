Motion Capture HRI simulation
=============================

MORSE has a number of new components to allow a human user to control an avatar
that can interact with the robot and some of the objects in the simulated
environment.

.. warning::
    This tutorial is currently oriented towards the experiments being carried
    out at LAAS for Human-Robot Interaction. They rely on the software packages
    of the `Openrobots suite <http://www.openrobots.org>`_, and on the scripts
    available in the `git repository <ssh://trac.laas.fr/git/robots/hri-simulation>`_.


Pre-requisites
--------------

These features require a Microsoft Kinect or ASUS Xtion and a Nintendo Wiimote,
to control the avatar.

The data from the Kinect/Xtion is read using the niut module in robotpkg. This
has to be installed following the usual instructions in
http://homepages.laas.fr/mallet/robotpkg/.

The Wiimote is used only to signal the simulation when we want to *take* an
object, while moving around in front of the Kinect/Xtion. Being wireless, it
allows for freedom of movement. We read input from this controller using
Bluetooth, and the `Cwiid <http://abstrakraft.org/cwiid/>`_ package.

In this tutorial, we use Pocolibs as middleware. We assume you have a
functional robotpkg installation will the necessary modules to control the Jido
robot.

Setup the scene
---------------

The scripts necessary to run this simulation can be obtaining from a git
repository::

$ git clone ssh://trac.laas.fr/git/robots/hri-simulation

We use a Builder script to generate the scene with the Jido robot, a human
avatar and some object that the robot can interact with. The script can be
found in the directory ``hri-simulation/scripts``, and is called
``hri.morse.mocap``.  Additionally, a Blender file is also available, which can
display the names of the different camera views. This file is called
``hri_mocap_fullscreen.blend``.

The script ``hri-simulation-mocap`` will launch MORSE, followed by the
necessary pocolibs modules: NIUT, SPARK, MHP, LWR, POM and TEXTOS.

Once MORSE is running, it is necessary to launch the helper program
``$MORSE_SOURCE/tools/wii_kinect_human_client.py``, and press the buttons
:kbd:`1` and :kbd:`2` on the Wiimote to make it discoverable. The 4 LEDs will
start blinking, and when only the led to the left is left on, the Wiimote has
been properly initialized. If all the LEDs turn off, you should relaunch the
script and try again to get it detected.

To get detected by the Xtion, it is necessary to stand in front of it, at about
2 meters away, and raise your arms in the Psy position.

Controlling the human
---------------------

Thanks to the Kinect/Xtion, the human avatar will emulate most of your
movements, as long as you remain within the field of view of the sensor.
The human is currently set to have no collisions, to avoid knocking down
objects inadvertently. This could be changed in the future.

Hold the Wiimote on your right hand, and use only the right hand to interact
with objects. When you want to pick up an object, place your hand close to it
(inside the object works fine) and hold the :kbd:`B` button on the Wiimote. As
long as you keep the button pressed you will hold the object. If you release
the button, the object will fall from your hand.

You can use the :kbd:`A` button on the Wiimonte to change the camera view
displayed in MORSE. In the current scene settings you can change between the
default scene view, the robot camera view and a first person view from the
perspective of the human. This last view can be useful to have a more immersive
experience and allows more intuitive manipulation of the objects.

Picking objects
---------------

You can add object that both the robot and the avatar can interact with,
following the instructions on :doc:`passive objects 
<../others/passive_objects>`.

For example, to add a corn flakes box into the scene, add the following lines
to the ``hri.morse.mocap`` file:

.. code-block:: python

    # Import, configure and place a static object from 'kitchen_objects.blend'.
    cornflakes = PassiveObject("props/kitchen_objects.blend", "Cornflakes")
    cornflakes.setgraspable()
    cornflakes.properties(Label = "My cornflakes")
    cornflakes.translate(-7, 3, 1.1)


.. image:: ../../../media/hri_cornflakes.jpg 
  :align: center

