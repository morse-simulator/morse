:orphan:

morse add manual page
=====================

Synopsis
--------

**morse add** [-h] [-f] {robot,sensor,actuator} name env

Description
-----------

Adds templates for a new component (sensor, actuator, robot)
to an existing simulation environment.

This component is meant to be a starting point for writing our own
custom components.

Options
-------

:{robot,sensor,actuator}:
                        the type of component you want to add.
:name:                  the name of the new component.
:env:                   the environment where to add.

:-h, --help:            show this help message and exit
:-f, --force:           forces the creation (possibly overwriting files).

Refer to :manpage:`morse(1)` for global MORSE options.

See Also
--------
:manpage:`morse(1)`
