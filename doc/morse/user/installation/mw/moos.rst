MOOS
~~~~

To build the MOOS middleware, you need to install MOOS and pymoos (MOOS Python bindings) on your system.

The instructions below install the new MOOS V10 release.  Information on installing previous versions of MOOS and pymoos can be found on their respective homepages: `MOOS homepage <http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php>`_ and `pymoos homepage <http://pymooos.sourceforge.net/>`_.

The pymoos bindings require at a minimum that the core MOOS packages be installed.  Pymoos is curently written for the pre-V10 header structure and so MOOS must be compiled with the ENABLE_V10_COMPATIBILITY flag set. To install the core MOOS components to your home directory::

    $ cd ~/
    $ git clone https://github.com/themoos/core-moos.git
    $ cd core-moos
    $ mkdir build
    $ cd build
    $ cmake ../ -DENABLE_V10_COMPATIBILITY=ON
    $ make
    
Pymoos also requires the Boost Python library compiled for Python 3.  The Boost versions shipped with newer versions of Ubuntu include both 2.7 and 3.2 versions of the Boost Python library and CMake will select the appropriate version.  

For older versions of Ubuntu (or other OS's), the latest version of the Boost source code can be downloaded from `Boost <http://http://www.boost.org>`_.  To install, extract the contents of the archive and run::

    $ ./bootstrap.sh --prefix=path/to/installation/prefix --with-python-version=3.2
    $ ./b2 install

Finally pymoos can be installed by::

    $ git clone git@github.com:davidhodo/pymoos.git
    $ cd pymoos
    $ mkdir build
    $ cmake ../
    $ make
    $ sudo make install

The installation can be verified by running 'import pymoos.MOOSCommClient' from a Python3 interpreter.
