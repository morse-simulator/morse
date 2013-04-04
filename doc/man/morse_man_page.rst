:orphan:

morse manual page
=================

Synopsis
--------

**morse** [-h] [-c] [--reverse-color] [-v] {create,rm,add,check,run,edit} ...

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

Options
-------

These options apply to any MORSE command.

:-c, --color:
        Uses colors for MORSE output. The default color scheme is well
        adapted to terminals with a dark background.
:--reverse-color:
        Uses an alternate color theme for MORSE output, well adapted to
        terminals with a bright background.
:-h, --help:
        Displays information regarding the program use.
:--version:
        Displays the version number.


Commands
--------

See "man morse-<command>" for a documentation of each commands.

:add:
        Adds templates for a new component (sensor, actuator, robot)
        to an existing simulation environment.
:check:
        Checks the environment is correctly setup to run morse.

:create:
        Creates a new simulation environment in the current directory.
        A template simulation scene is also created.
        The environment is added to 'sites' in ~/.morse/config
:edit:
        Open the given Blender scene or Python script in the Blender
        interface for edition. The simulation can be started by 
        pressing P.
:rm:
        Deletes an existing simulation environment.
:run:
        Runs a simulation (must be a Python script) without loading 
        the Blender interface.

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
