Basic Blender commands 
======================

The global environment used within MORSE is based on Blender.
While it is not necessary to know a lot of Blender to use MORSE, it can be useful
when tweaking existing objects, for example, when adjusting the
positions of the elements on a scene. Here we present the most useful
Blender commands for use with MORSE.

Blender's user interface is very unusual, and works differently from
most other graphical user interface programs.
Follow this link for a good tutorial on `the use of Blender's interface <http://www.blendercookie.com/getting-started-with-blender/>`_.
Here we present the most useful commands necessary to operate MORSE.

.. note:: The Blender interface is divided in various panels (windows).
    The mouse and keyboard shortcuts vary depending on the panel over
    which the mouse pointer is located.
    All the following commands are valid for when the **mouse pointer is located
    inside the 3D View panel**.

Object Selection
----------------

:kbd:`Right Mouse Click` the object

Pressing :kbd:`a` will toggle selection of all or none of the objects in the current scene.

Object Geometric Transformations
--------------------------------

Keyboard input
++++++++++++++

To enter exact transformation values with the keyboard, press :kbd:`N` to display the
**Transform Properties** sub-window for the selected object.
Then change the values of the **Location**, **Rotation** or **Scale** fields.

With the mouse
++++++++++++++

The shortcut keys are :kbd:`g` for translation, :kbd:`s` for scaling and :kbd:`r` for rotation.
Press the desired key, then adjust the values by moving the mouse.
Press :kbd:`Left Mouse Click` to accept transformation, or :kbd:`Right Mouse Click` to cancel.

Tips:

- Hold down the :kbd:`Ctrl` key while transforming, to use discrete intervals. 
- Hold down the :kbd:`Shift` key to make more precise transformations (mouse movements produce smaller changes)
- Press :kbd:`x`, :kbd:`y` or :kbd:`z` immediately after the transformation shortcut 
    to constrain the transformation to one axis.

Camera view shortcuts
---------------------

Use the numeric keyboard:

- Top view: :kbd:`7`
- Front view: :kbd:`1`
- Side view: :kbd:`3`
- Toggle main camera view: :kbd:`0`
- Toggle perspective/orthogonal view: :kbd:`5`

The view of the scene can also be adjusted manually by holding down :kbd:`Middle Mouse Click` and moving the mouse.
Panning around the scene is done by holding down :kbd:`Shift` key, holding the :kbd:`Middle Mouse Click` and moving the mouse.


Switch shading modes
--------------------

- Wireframe/solid: :kbd:`z`
- Wireframe/solid: :kbd:`Shift-z`
- Solid/textured: :kbd:`Alt-z`

Opening and saving files
------------------------

- Open file: :kbd:`F1`
- Save file: :kbd:`Ctrl-w`
- Save file as... : :kbd:`F2`
- Load a new file: :kbd:`Ctrl-n`, then :kbd:`Enter`
- Quit Blender: :kbd:`Ctrl-q`, then :kbd:`Enter`
