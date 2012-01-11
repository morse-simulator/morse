:orphan:

morse manual page
========================

Synopsis
--------

**morse** [command] [options]


Description
-----------
Morse, the Modular OpenRobots Simulation Engine, uses the Blender Game Engine
to provide a general purpose robot simulator.
A robot and its environment are defined in Morse by creating a model in
Blender. Actuator and sensor objects are attached to a model, and may be
interacted with via a variety of middleware protocols. Simulations can be
programmed via Python scripts or inside of Blender itself. Currently supported
middleware includes YARP, MOOS, ROS, Pocoslibs, as well as a plain socket
interface.

Commands
--------
:[None]:
        Launchs the simulator interface with a default scene
:create filename:
        Creates a new empty scene and launchs the simulator interface
:run filename:
        Runs a simulation without loading the simulator interface
:exec filename:
        Runs the given Python script with a default scene
:check:
        Checks the environment is correctly setup to run morse
:help:
        Displays information reguarding the program
:version:
        Displays the version number

See Also
--------
:manpage:`morseexec(1)` :manpage:`blender(1)`
