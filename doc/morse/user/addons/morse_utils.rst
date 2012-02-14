Setting up a Scene with MORSE Utils
===================================

This add-on creates the new menu ``Morse Utils`` in Blender's ``Logic Editor`` window.
It helps you make your scene an interactable Environment.

Objects
-------

To set up the object, just select it and press the Button ``Morse Object``.
A dialogue window will appear with the a bunch of items. See :doc:`passive objects 
<../other/passive_objects>`.
In addition you can also set the objects ``mass`` and ``collision bounds`` of the object.

Drawers
-------

Select the drawer and press ``Morse Drawer``. The items are:
- **Drawer**: Description or name.
- **Description**: A short description or name. Will be displayed in :doc:`human's manipulation mode <human>`.
- **End Frame**: Determines the end of the animation in Frames. Default Animation Rate is 24 Frames per second.
- **Open**: Check if the drawer's initial state is open.
- **Generate Action**: Generate simple animations along a (global) axis. If set to ``No``, you have to animate the
    drawer manually and assign this action to the actuators ``Open`` and ``Close``.

Doors
-----

Making Doors interactable is a bit different. Again assign the needed properties with the dialogue window:
- **Door**: The side where the door's hinge is.
- **Description**: A short description or name. Will be displayed in :doc:`human's manipulation mode <human>`.
- **Open**: Check if the drawer's initial state is open.

In addition you have to set the door's origin to the hinge. To do so, set the ``3D Cursor`` to where the hinge is.
This can be done via ``3D View -> Properties -> 3D Cursor`` (toggle ``Properties`` with ``N``). Alternatively use 
``Tab`` to enter ``Edit Mode``, select ``Vertices/Edges`` so that their center is the hinge and press ``Shift - S``.
Select ``Cursor to Selected`` and leave ``Edit Mode``. With the ``3D Cursor`` set, hit ``Ctrl - Shift - Alt - C`` or 
``3D View -> Tool Shelf -> Object Tools -> Origin`` (toggle ``Tool Shelf`` with ``T``) and choose ``Origin to 3D Cursor``.

Mechanical and Electric Devices
-------------------------------

Simulated devices need a switch to turn them on and off. Select the switch and run the ``Morse Switch`` dialogue. Select 
the device in the ``Switch`` Dropdown menu. In Blender Versions < 2.61 you have to manuelly type the name. Change ``On`` 
for the devices initial state.

The effect the device has at runtime is up to you. 
However there is a preset for lights which can only be used for ``Lamp`` Objects. The ``Energy`` equals Blender's Lamp Units.

