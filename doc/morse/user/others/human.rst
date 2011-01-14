The human component in MORSE
============================

MORSE allows the simulation of humans: you can add a human model in your scene, you can
control it from the keyboard and mouse during the simulation (move it around, sit it 
down, pick and place objets...), and export to your robotic systems various data (currently
only the position and joint state, but other higher level states like gesture, grasped
objects,... are planned).

.. image:: ../../../media/hri_import_human.jpg
   :align: center

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/human/default_human.blend``
- Python: 

  - ``$MORSE_ROOT/src/morse/robots/human.py``
  - ``$MORSE_ROOT/src/morse/blender/human_head.py``
  - ``$MORSE_ROOT/src/morse/blender/human_control.py``
  - ``$MORSE_ROOT/src/morse/blender/human_stance.py``

Human control
-------------

The human is meant to be used in *immersive* mode (so-called *first-person 
shooter* view): once imported, you must define the human camera as the default 
camera.

Do so by selecting the camera on the human's head, and press :kbd:`Ctrl-Numpad 0`
(or menu ``View > Cameras > Set Active Object As Camera``).

Motion mode
~~~~~~~~~~~

- Move the character with the :kbd:`W`, :kbd:`A`, :kbd:`S`, :kbd:`D`  keys

.. warning:: The default Blender scene comes with a flighing camera (``CameraFP``)
    which is mapped to the same keys. A simple workaround consists in simply 
    deleting the ``CameraFP`` object.
    
- The direction of the head is controlled with the movement of the mouse, via
  and IK target (the yellow ball).

.. warning:: The head may be shaky when moving the mouse: this is due to numerical
    instability in the IK solver, and only appear in specific Blender build. We
    are investigating this issue.

- To make the character sit, press :kbd:`C`.

Manipulation mode
~~~~~~~~~~~~~~~~~

To toggle in and out of manipulation mode, press :kbd:`X`.

When in manipulation mode, the camera and the hand are controlled with 
the mouse. You can also make the hand move closer or farther with the 
scroll wheel.

In manipulation mode, press and hold the :kbd:`Left Mouse Button` to take an 
item, then release to let go of the object.

.. note:: Only objects with a specific game property called ``Object`` can be
    carried by the human.

