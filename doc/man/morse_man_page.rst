:orphan:

morse manual page
=================

Synopsis
--------

**morse** [-h] [-v] [--name NAME] [--reverse-colors] [-c] [-g GEOM]
             {check,edit,run} [scene] ...


Description
-----------
MORSE, the Modular OpenRobots Simulation Engine, is a general-purpose robotics
simulator, primarily intended for an academic audience.

It relies on the Blender Game Engine to provide a semi-realistic 3D 
environment with physics simulation.

A robot and its environment are created in MORSE by building a model in
Blender. Actuator and sensor objects are attached to robot models, and may be
interacted with via a variety of middleware protocols. Simulations can be
programmed via Python scripts or inside of Blender itself. Currently supported
middleware includes YARP, MOOS, ROS, Pocoslibs, as well as a plain socket
interface.

Commands
--------

:check:
        Checks the environment is correctly setup to run morse.
:edit filename:
        Open the given Blender scene or Python script in the Blender
        interface for edition. The simulation can be started by 
        pressing P.
:run filename:
        Runs a simulation (must be a Python script) without loading 
        the Blender interface.
:base blender_file:
        When running in Edit mode, and assuming your passing a BuilderAPI 
        Python script, you can optionally set a 'base' Blender file on 
        which the Python script is applied. This is convenient to quickly
        change the simulated environment wihtout touching the Python 
        script, for instance.
:name node_name:
        When running in multi-node mode, sets the name of this
        node. It defaults to current hostname.
:color:
        Uses colors for MORSE output. The default color scheme is well
        adapted to terminals with a dark background.
:reverse-color:
        Uses an alternate color theme for MORSE output, well adapted to
        terminals with a bright background.
:geometry WxH+dx,dy:
        Sets the simulator window geometry. Expected format: either WxH 
        or WxH+dx,dy to set an initial x,y delta (from the lower left 
        corner).
:help:
        Displays information regarding the program use.
:version:
        Displays the version number.

See Also
--------
:manpage:`morseexec(1)` :manpage:`blender(1)`
