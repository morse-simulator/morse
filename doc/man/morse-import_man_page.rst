:orphan:

morse import manual page
========================

Synopsis
--------

**morse import** [-h] [-f] path [name]

Description
-----------

Imports a directory as a simulation environment, ie, adds an entry to MORSE's
"sites" section of the configuration file (typically, $HOME/.morse/config) that
refers to this directory.

Options
-------

:path:          a relative or absolute path to the simulation
                environment to import.
:name:          the name given to the imported environment.
                If not specified, default to the name of the
                containing directory (path's basename).

:-h, --help:   show this help message and exit
:-f, --force:  forces the import (possibly overwriting existing entry).

Refer to :manpage:`morse(1)` for global MORSE options.

See Also
--------
:manpage:`morse(1)`
