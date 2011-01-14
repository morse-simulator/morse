Human-Robot interaction tutorial
================================

This tutorial shows how to build a simulation with a robot and a human that
is interactively controlled by the simulation user.

In this simple scenario, the robot is ordered to follow the human.

Pre-requisites
--------------

- You must have completed the :doc:`first tutorial <../tutorial>`.
- In this tutorial, we use ROS as middleware. We assume you have a functionnal 
ROS installation (you should only need the core ROS tools). If you need to
install ROS, please refer to `ROS installation instruction <http://www.ros.org/wiki/ROS/installation>`_.

Setup the scene
---------------

Launch MORSE. We will use the default indoor scene, so we can keep it as it appears.

We first want to add a human:

#. With the mouse over the 3D view in Blender, press :kbd:`Ctrl-Alt-O` to open the Load Library browser
#. Navigate to the directory ``$MORSE_ROOT/data/morse/components/humans``
#. Press :kbd:`Left Mouse Click` over the file ``default_human.blend``
#. Press :kbd:`Left Mouse Click` over the item ``Object``
#. Press :kbd:`Right Mouse Click` and drag over the names of all the objects listed, to select them all
#. Press the button **Link/Append from Library**. You'll return to the 3D View, and the newly added 
human is selected.
#. Convert the objects to local: without de-selecting the object, press :kbd:`l` then hit :kbd:`enter`

.. image:: ../../../media/hri_import_human.jpg
   :align: center

Controlling the human
---------------------

To avoid keyboard mapping issues, remove the ``CameraFP`` object by selecting it in the scene outliner, and
pressing :kbd:`del`. Set the human camera as the new *main* camera: select it and press :kbd:`Ctrl-Numpad 0`
(or menu ``View > Cameras > Set Active Object As Camera``).

You can control the human with the :kbd:`W`, :kbd:`A`, :kbd:`S`, :kbd:`D`  keys.

Press :kbd:`P` to start the simulation and move around. When moving the mouse, you displace the yellow IK
target of the head. This allows you to control the head direction.

Picking objects
---------------
Our human can pick and release objects. Let's add a new cube on one of the tables: switch to the ortho
view (:kbd:`Numpad 5`), and using the front view (:kbd:`Numpad 1`) and top view (:kbd:`Numpad 7`), place
the 3D cursor on the top of a table:

.. image:: ../../../media/place_3dcursor.jpg 
  :align: center

Push :kbd:`space` and type ``Add cube`` followed by :kbd:`enter`. Using ``Scale`` (:kbd:`S` key) and
``Translate along Z`` (:kbd:`G` fllowed by :kbd:`Z`), turn the box into a reasonably sized object:

.. image:: ../../../media/place_cube.jpg 
  :align: center

To allow this object to be grasped, add a custom game property (of any type) to your box named ``Object``:

.. image:: ../../../media/gameproperty_object.jpg 
  :align: center

Start again the simulation, and press the :kbd:`X` key to switch to the manipulation mode. You can control
the hand with the mouse and the scroll wheel. Press the :kbd:`Left Mouse Button` when near an object to
pick it, and release the button to place the object.

.. image:: ../../../media/hri_pick_object.jpg 
  :align: center

Check the :doc:`human component <../others/human>` documentation for more details on what can be done
with the human component.

Exporting the human position
----------------------------

