:orphan:

morse run manual page
=====================

Synopsis
--------

**morse run** [-h] [--name NAME] [-g GEOM] [env] [file] [pyoptions...]

Description
-----------

Runs a simulation (must be a Python script) without loading 
the Blender interface.

Options
-------

:env:
                  the simulation environment to run.
:file:
                  the exact scene (.py or .blend) to run (if 'env' is
                  given, within this environment).
                  See section FILE RESOLUTION below for details.

:pyoptions:
                  optional parameters, passed to the Blender python
                  engine in sys.argv
:-h, --help:
            show this help message and exit

:--name NAME:
            when running in multi-node mode, sets the name of this
            node (defaults to either MORSE_NODE if defined or
            current hostname).
:-g GEOM, --geometry GEOM:
            sets the simulator window geometry. Expected format:
            WxH or WxH+dx,dy to set an initial x,y delta (from
            lower left corner).

Refer to :manpage:`morse(1)` for global MORSE options.

File Resolution
---------------

MORSE tries to figure out which simulation you want to open with the following strategy:


**If only one parameter *arg* is given:**

- if *arg* is a configured simulation environment with prefix $ENVROOT:
   - if *$ENVROOT/default.py* exists, launch it.
   - else, if any file with an extension {.py|.blend} exists, launch the first
     one (in alphanumerical order) 
   - else if a file called *arg* exists in the current directory, launch it.
   - else, fail
- else check if a file called *arg* exists, and launch it (note that in that
  case, *arg* can contain an absolute path or a path relative to the current 
  directory).

**If two parameters *arg1* and *arg2* are given:**

- if *arg1* is a configured simulation environment with prefix $ENVROOT:
    - if *$ENVROOT/arg2* exists, launch it
    - else, add $ENVROOT to MORSE environment, and if *arg2* exists, launch it (note
      that *arg2* can contain an absolute path or a path relative to the current 
      directory)
- else fail


See Also
--------
:manpage:`morse(1)`
