MOOS
~~~~

To build the MOOS middleware, you need to install MOOS and ``pymoos`` (MOOS Python bindings) on your system.

The instructions below install the new MOOS V10 release.  Information on
installing previous versions of MOOS and ``pymoos`` can be found on their respective
homepages: `MOOS homepage
<https://sites.google.com/site/moossoftware/>`_ and `pymoos homepage
<https://github.com/msis/python-moos>`_.

The ``pymoos`` bindings require at a minimum that the core MOOS packages be
installed::

    $ cd ~/
    $ git clone https://github.com/themoos/core-moos.git
    $ cd core-moos
    $ mkdir build
    $ cd build
    $ cmake ../
    $ make

Finally ``pymoos`` can be installed by::

    $ git clone https://github.com/msis/python-moos pymoos
    $ cd pymoos
    $ mkdir build
    $ cmake ../ -DPYBIND11_PYTHON_VERSION=3
    $ make
    $ sudo make install

Because ``pymoos`` supports both major versions of Python,
it is necessary to make sure that version 3 is the one selected.

The installation can be verified by running ``import pymoos``
from a Python3 interpreter ::

    $ python3
    >>> import pymoos

If you receive and ``ImportError``, then you need to add the installation path
to ``PYTHONPATH`` ::

    $ export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages/

PS :: The installation path is printed with the install command.
