The human component in MORSE
============================

MORSE allows the simulation of humans: you can add a human model in your scene, you can
control it from the keyboard and mouse during the simulation (move it around, sit it 
down, pick and place objects...), and export to your robotic systems various data (currently
only the position and joint state).

The human is handled inside MORSE as a robot, which means it can have sensors and actuators
attached to it.

.. image:: ../../../media/hri_import_human.jpg
   :align: center
   :width: 400px

For a general introduction to human-robot interaction simulation with MORSE, check the
:doc:`HRI main page <../../hri>`.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/human/human.blend``
- Python: 

  - ``$MORSE_ROOT/src/morse/robots/human.py``
  - ``$MORSE_ROOT/src/morse/blender/human_interaction/*``

Inserting the human in your scene
---------------------------------

The procedure to insert a human in your simulation is slightly different than for
other assets.

From Blender interface
++++++++++++++++++++++

You can append a human while editing a scenario in Blender by importing the ``Human``
**group** available in ``$MORSE_ROOT/data/morse/human/human.blend``.

If the model do not appear, it has probably be added to an invisible layer. Check in
the layers panel.

With the Builder API
++++++++++++++++++++

To add a human with the :doc:`MORSE Builder API <../../dev/builder>`, you just need
to instantiate the :py:class:`morse.builder.morsebuilder.Human` class.

.. code-block:: python

   from morse.builder.morsebuilder import *
   human = Human()

The :doc:`human posture component <morse/user/sensors/human_posture>`
that can be accessed through the ``armature`` member.

Usage example:

.. code-block:: python

   #! /usr/bin/env morseexec

   from morse.builder.morsebuilder import *

   human = Human()
   human.translate(x=5.5, y=-3.2, z=0.0)
   human.rotate(z=-3.0)

   human.armature.configure_mw('pocolibs',
                    ['Pocolibs',
                     'export_posture',
                     'morse/middleware/pocolibs/sensors/human_posture',
                     'human_posture'])


Human control
-------------

When starting a simulation with a human, the active camera is automatically
set behind the human, in *immersive* mode (so-called *first-person 
shooter* view). You can switch to another camera with the :kbd:`F9` key.


Motion mode
+++++++++++

.. image:: ../../../media/hri_move_mode.jpg
   :align: center
   :width: 400px


The human always starts in so-called **Motion Mode**.

- Move the character with the :kbd:`W`, :kbd:`A`, :kbd:`S`, :kbd:`D` keys
    
- The direction of the head is controlled with the movement of the camera.

- To make the character sit, press :kbd:`C`.

Manipulation mode
+++++++++++++++++

.. image:: ../../../media/hri_manipulation_mode.jpg
   :align: center
   :width: 400px


To toggle in and out of **Manipulation Mode**, press :kbd:`X`.

In manipulation mode, when the hand is close enough of a graspable object (see
:doc:`passive objects <passive_objects>` documentation to know how to define a
graspable object), a label ``Pickup the object`` appears. Press the :kbd:`Left
Mouse Button` to take the item, and :kbd:`Right Mouse Button` to release it.


Sensors and actuators
---------------------

Currently (``morse-0.5``), the human component has one sensor already
integrated, which exports the joint state of the human: the :doc:`human posture
sensor <../sensors/human_posture>`.  This sensor is already embedded in the
``human.blend`` file. You don't need to link it from a separate sensor file.

The human model can be controlled using the keyboard, or have a motion
controller attached to it, so that the movement commands can come from an
external software.

Services
--------

- **move**: (Synchronous service) Move the body, or the hand in case of
  manipulation mode. In case of moving the body, the two arguments expected are
  speed and rotation. In case of moving the hand these two arguments should be
  X and Z displacement.

- **move_head**: (Synchronous service) Move the head. Two arguments expected:
  pan and tilt
    
- **grasp_**: (Synchronous service) Grasp and release an object if in
  manipulating mode. Takes one argument: must be "t" to grasp and "f" to
  ungrasp
    
- **move_hand**: (Synchronous service) Move the hand in the third direction (
  Y, see above). 
    
- **toggle_manipulation**: (Synchronous service) Switch from and to
  manipulation mode
    
