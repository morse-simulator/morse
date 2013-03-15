:orphan:

morse manual page
=================

Synopsis
--------

**morse** [-h] [-b BASE] [--name NAME] [-c] [--reverse-color] [-g GEOM]
          [-v] {create,check,edit,run} [env] [scene] ...


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
middleware includes YARP, MOOS, ROS, Pocolibs, as well as a plain socket
interface.

Commands
--------

:check:
        Checks the environment is correctly setup to run morse.
:create env:
        Creates a new simulation environment in the current directory.
        A template simulation scene is also created.
        The environment is added to 'sites' in ~/.morse/config
:edit [env] [scene]:
        Open the given Blender scene or Python script in the Blender
        interface for edition. The simulation can be started by 
        pressing P.
        If only 'env' is set, open 'scene.py' (or the first .py or .blend 
        found if 'scene.py' does not exist) in the given environment.
        If only [scene] is set, try to open this file in the current
        directory.
        If both are set, open the file 'scene' in the environment 'env'.
:run [env] [scene]:
        Runs a simulation (must be a Python script) without loading 
        the Blender interface.
        See above for the meaning of the [env]  and [scene] parameters.
:base blender_file:
        When running in Edit mode, and assuming your passing a Builder API 
        Python script, you can optionally set a 'base' Blender file on 
        which the Python script is applied. This is convenient to quickly
        change the simulated environment without touching the Python 
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


Files
-----

Configuration files are stored in each user $HOME/.morse

:config:
        General MORSE configuration.
        Section 'sites' contains the list of simulation environments
        MORSE will look for at startup.

Environment
-----------

:MORSE_ROOT:
			Use this variable to determine where are localised data. This
			variable is mandatory

:MORSE_BLENDER:
			Determine which blender binary is started. If it does not exist, 
			rely on the first blender in the PATH

:MORSE_RESOURCE_PATH:
			Determine where morse will search for blender components. It is a
			colon-separated list of directories, similar to PATH. 

:MORSE_NODE:
			In multi-node mode, if no name has been given, look for this
			variable to determine the name of the node. If it does not exist,
			rely on the name of the host.

Morse relying on Python to execute itself, the run of Morse is influenced by
all Python variables, in particular **PYTHONPATH**. See :manpage:`python(1)` for
details.

See Also
--------
:manpage:`morseexec(1)` :manpage:`blender(1)` :manpage:`python(1)`
