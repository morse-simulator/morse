MOOS
~~~~

To build the MOOS middleware, you need to install MOOS and pymoos on your system.

Additional information on MOOS and pymoos can be found at `MOOS homepage <http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php>`_ and `pymoos homepage <http://pymooos.sourceforge.net/>`_.

To install MOOS to your home directory::

    $ cd ~/
    $ svn co svn://login2.robots.ox.ac.uk/MOOS/trunk MOOS
    $ cd MOOS
    $ cmake .
    $ make
    
Pymoos requires the Boost Python library compiled for Python 3.  The binaries available in most repositories are currently compiled for version 2.7.   The latest version of the Boost source code (currently 1.47)  can be downloaded from `Boost <http://http://www.boost.org>`_.  To install::

    $ ./bootstrap.sh --prefix=path/to/installation/prefix --with-python-version=3.2
    $ ./b2 install

Finally pymoos can be installed by::

    $ cd ~/
    $ svn co https://pymooos.svn.sourceforge.net/svnroot/pymooos pymoos
    $ cd pymoos
    $ cmake .
    $ make
    $ sudo make install
    
When running ``cmake`` for pymoos make sure to select the MOOS support option.
