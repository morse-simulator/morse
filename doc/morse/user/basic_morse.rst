Getting started: basic MORSE usage 
==================================

Basic Blender commands 
----------------------

The setup and configuration of a simulation scenario in MORSE is completely done using the Blender interface.
It is very particular and different from almost any other computer program available.
Follow this link for good tutorial on `the use of Blender's interface <http://www.blendercookie.com/getting-started-with-blender/>`_.
Here we present the most useful commands necessary to operate MORSE.

.. note:: The Blender interface is divided in various panels (windows).
    The mouse and keyboard shortcuts used change depending on the panel over
    which the mouse pointer is located.
    All the following commands are valid for when the **mouse pointer is located
    inside the 3D View panel**.

Selection of an object 
++++++++++++++++++++++

:kbd:`Right Mouse Click` over the object

Pressing :kbd:`a` will toggle selection of all or none of the objects in the current scene.

Geometric transformations of an object
++++++++++++++++++++++++++++++++++++++

Keyboard input
~~~~~~~~~~~~~~

To enter exact transformation values with the keyboard, press :kbd:`N` to display the
**Transform Properties** sub-window of the selected object.
Then change the values of the **Location**, **Rotation** or **Scale** fields.

With the mouse
~~~~~~~~~~~~~~

The shortcut keys are :kbd:`g` for translation, :kbd:`s` for scaling and :kbd:`r` for rotation.
Press the desired key, then adjust the values by moving the mouse.
Press :kbd:`Left Mouse Click` to accept transformation, or :kbd:`Right Mouse Click` to cancel.

Tips:

- Hold down :kbd:`Ctrl` key while transforming, to use discrete intervals. 
- Hold down :kbd:`Shift` key to make more precise transformations (mouse movements produce smaller changes)
- Press :kbd:`x`, :kbd:`y` or :kbd:`z` immediately after the transformation shortcut 
    to constraint the transformation on one axis.

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




MORSE controls
--------------

A simulation in MORSE is executed using the Game Engine mode of Blender. When in this mode, the button panels of Blender are disabled and the controls change.
It is recommended to switch the Blender 3D View window to full screen before starting a simulation. Press :kbd:`Ctrl-Up` when the mouse is over a window to toggle it to full screen and back.

Simulation camera control
+++++++++++++++++++++++++

The default MORSE scene, as well as most of the provides scenario files, contain an object called **CameraFP**.
It allows the user to control the view of the scene during the simulation, using the mouse and keyboard,
in the same way as First Person Shooter game. The camera can be moved around using:
    
- :kbd:`w`/:kbd:`z`, :kbd:`s` to move forward and backward
- :kbd:`a`/:kbd:`q`, :kbd:`d` to move left and right
- :kbd:`r`, :kbd:`f` to move up and down

The direction the camera points to is controlled with the mouse. Hold down the :kbd:`Ctrl` key while moving the mouse.

It is possible to adjust the speed of the camera movement, by selecting the **CameraFP** object in the scene,
and adjusting the game properties of the camera object: **Sensitivity** for the mouse, and **Speed** for the keyboard.


Standard keyboard functions
+++++++++++++++++++++++++++

The following are key bindings already defined in any simulation scene

- :kbd:`p` key: start the simulation (initiate the Game Engine)

- :kbd:`h` key: show an on screen display with this list of keyboard functions

- :kbd:`Esc` key: stop and exit the simulation (cleaning up the connections)

- :kbd:`F9` key: cycle through the camera views of all Blender camera objects in the scene.
    This includes the cameras mounted on robots, as well as the default **CameraFP**

- :kbd:`F11` key: reset all objects to their original position at the start of the simulation

.. warning::  
  There is a bug in the code, which will make any forces acting on an object be still active when
  the position is reset. This can cause an object to start rotating or moving without an apparent reason.

- :kbd:`F12` key: emergency exit from the simulation, without clean up. Useful when the :kbd:`Esc` key
    does not make the simulation finish (generally happens when the simulation could not properly initialise)

.. note::  
  If this doesn't work either, switch to the command terminal where MORSE was started and type: :kbd:`Ctrl-\\`.
  This will kill Blender.




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
++++++++++++++++++++++++++

#. Open file
#. Link elements (robots, scenarios sensors) :kbd:`Ctrl-Alt-o`
#. Select the source Blender file, and then Objects
#. Select the objects to insert, using the :kbd:`Right Mouse Click`
#. Click **Link/Append from Library**
#. The objects will be inserted in the scene, at the origin, and with a cyan selection highlight
#. If the inserted object has to be placed in a different location, it must be made local. Press the :kbd:`l` key, then select **Selected Objects** or press :kbd:`Enter`
#. The object will now have a pink selection highlight, and can be moved/rotated/scaled

Parenting components to a robot
+++++++++++++++++++++++++++++++

- Select the Empty object of the component, then hold :kbd:`Shift` and select the robot
- Press :kbd:`Ctrl-p`, then select **Make parent** or press :kbd:`Enter`

Using middlewares
+++++++++++++++++

To use one of the available middleware bindings import/export data from the simulated actuators/sensors:

#. Link a middleware object, as explained above
#. Edit the ``component_config.py`` script to indicate what middleware will be attached to each component, as explained in the :doc:`hooks <hooks>` section
