:orphan:

morse import manual page
========================

Synopsis
--------

**morse import** [-h] [-f] path [name]

Description
-----------

Imports a directory as a simulation environment, i.e., adds an entry to MORSE's
configuration file's "sites" section that refers to this directory.
(The configuration file is usually $HOME/.morse/config)

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
