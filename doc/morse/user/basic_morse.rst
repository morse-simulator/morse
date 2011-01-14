Getting started: basic MORSE usage 
==================================

Basic Blender commands 
----------------------

The setup and configuration of a simulation scenario in MORSE is completely done using the Blender interface. It is very particular and different from almost any other computer program.
A good explanation on the use of Blender's interface can be found `here <http://en.wikibooks.org/wiki/Blender_3D:_Noob_to_Pro/Learning_Graphic_Interface>`_.
Here we present the most useful commands necessary to operate MORSE

Selection of an object 
++++++++++++++++++++++

:kbd:`Right Mouse Click` over the object

Pressing :kbd:`a` will toggle selection of all or none of the objects in the current scene.

Geometric transformations of an object
++++++++++++++++++++++++++++++++++++++

Keyboard input
~~~~~~~~~~~~~~

To enter with the keyboard accurate transformation value, press :kbd:`N` to display the **Transform Properties** sub-window of the selected object. Then change the values of the **Location**, **Rotation** or **Scale** fields.

With the mouse
~~~~~~~~~~~~~~

The shortcut keys are :kbd:`g` for translation, :kbd:`s` for scaling and
 :kbd:`r` for rotation.
 
Press the desired key, then adjust the values by moving the mouse.

Press :kbd:`Left Mouse Click` to accept transformation, or :kbd:`Right Mouse Click` to cancel.

Tips:

- Hold :kbd:`Ctrl` key while transforming, to use discrete intervals. 
- Press :kbd:`x`, :kbd:`y` or :kbd:`z` to constraint the transformation on one axis.

Camera view shortcuts
+++++++++++++++++++++

Use the numeric keyboard:

- Top view: :kbd:`7`
- Front view: :kbd:`1`
- Side view: :kbd:`3`
- Toggle main camera view: :kbd:`0`
- Toggle perspective/orthogonal view: :kbd:`5`

The view of the scene can also be adjusted manually by holding down :MMB: and moving the mouse.
Panning around the scene is done by holding down :kbd:`Shift` key, holding the :MMB: and moving the mouse.


Switch shading modes
++++++++++++++++++++

- Wireframe/solid: :kbd:`z`
- Wireframe/solid: :kbd:`Shift-z`
- Solid/textured: :kbd:`Alt-z`

Opening and saving files
++++++++++++++++++++++++

- Open file: :kbd:`F1`
- Save file: :kbd:`Ctrl-w`
- Save file as... : :kbd:`F2`
- Load a new file: :kbd:`Ctrl-n`, then :kbd:`Enter`
- Quit Blender: :kbd:`Ctrl-q`, then :kbd:`Enter`




MORSE camera view and control
-----------------------------

A simulation in MORSE is executed using the Game Engine mode of Blender. When in this mode, the button panels of Blender are disabled and the controls change.
It is recommended to switch the Blender 3D View window to full screen before starting a simulation. Press :kbd:`Ctrl-Up` when the mouse is over a window to toggle it to full screen and back.

During simulation, the view of the scene is controlled using the mouse and keyboard, in the same way as First Person Shooter game.The camera can be moved around using:
    
- :kbd:`w`/:kbd:`z`, :kbd:`s` to move forward and backward
- :kbd:`a`/:kbd:`q`, :kbd:`d` to move left and right
- :kbd:`r`, :kbd:`f` to move up and down

The direction the camera points to is controlled with the mouse.

It is possible to adjust the speed of the movement, by selecting the ''CameraFP'' object in the scene, and adjusting the properties **Sensitivity** for the mouse, and **Speed** for the keyboard.


Creation of scenario files
--------------------------

Use the ''morse'' binary program, to open or create new simulation scenarios:

  $ morse

Will create a new copy of the basic scenario file, ready to be modified.

  $ morse create [Filename]

It will create a new file with the name given, and ready to be edited.

  $ morse [Filename]

The MORSE simulator is started using an existing file with the name specified.

Adding components to files
--------------------------

#. Open file
#. Link elements (robots, scenarios sensors) :kbd:`Ctrl-Alt-o`
#. Select the source Blender file, and then Objects
#. Select the objects to insert, using the :kbd:`Right Mouse Click`
#. Click **Link/Append from Library**
#. The objects will be inserted in the scene, at the origin, and with a cyan selection highlight
#. If the inserted object has to be placed in a different location, it must be made local. Press the :kbd:`l` key, then select **Selected Objects** or press :kbd:`Enter`
#. The object will now have a pink selection highlight, and can be moved/rotated/scaled

Parenting components to a robot
-------------------------------

- Select the Empty object of the component, then hold :kbd:`Shift` and select the robot
- Press :kbd:`Ctrl-p`, then select **Make parent** or press :kbd:`Enter`

Start and stop a simulation
---------------------------

To start the simulation, press the :kbd:`p` key.

To stop, press the :kbd:`Esc` key.

.. note::  
  Under certain error conditions :kbd:`ESC` may not work, in that case, an emergency exit
  key is :kbd:`F12`, which will try to exit without doing executing the 
  cleanup functions. If this doesn't work, the command :kbd:`Ctrl-\\` on the 
  terminal will kill Blender.
