:orphan:

morse edit manual page
======================

Synopsis
--------

**morse edit** [-h] [-b BASE] [--name NAME] [-g GEOM] [env] [file] [pyoptions...]

Description
-----------

Open the given Blender scene or Python script in the Blender
interface for edition. The simulation can be started by 
pressing P.

Options
-------

:env:                   the simulation environment to edit.
:file:                  the exact scene (.py or .blend) to edit (if 'env' is
                        given, within this environment).
                        See section FILE RESOLUTION of :manpage:`morse-run(1)` for details.
:pyoptions:             optional parameters, passed to the Blender python
                        engine in sys.argv

:-h, --help:            show this help message and exit
:-b BASE, --base BASE:  specify a Blender scene used as base to apply the
                        Python script.
:--name NAME:           when running in multi-node mode, sets the name of this
                        node (defaults to either MORSE_NODE if defined or
                        current hostname).
:-g GEOM, --geometry GEOM:
                        sets the simulator window geometry. Expected format:
                        WxH or WxH+dx,dy to set an initial x,y delta (from
                        lower left corner).

Refer to :manpage:`morse(1)` for global MORSE options.

See Also
--------
:manpage:`morse(1)`
