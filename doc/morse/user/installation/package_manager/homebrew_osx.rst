With Homebrew on OSX
++++++++++++++++++++

`Homebrew <http://brew.sh>`_ is a package manager for OSX.
It can be installed in OSX with a single Ruby command::

    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"


.. note::

    Homebrew requires XCode and the command line tools to be installed.

Installing MORSE
----------------

#. Download `Blender <http://www.blender.org/download/get-blender/>`_ and
   copy the apps to ``/Applications``. Blender is not currently available
   as a recipe in Homebrew.

   .. note::
    ``MORSE_BLENDER`` should be added to your .bashrc and must point to the
    actual executable, not the app directory::

            MORSE_BLENDER=/Applications/Blender.app/Contents/MacOS/blender

#. Check which version of Python is used by Blender. For that purpose, in a
   console, 

   .. code-block:: sh

        echo 'import sys; print("VERSION %s" % sys.version.split()[0]); sys.exit(0)' > /tmp/version.py
        ${MORSE_BLENDER} -y -P /tmp/version.py 2>&1 | grep VERSION


#. Once you get the version used by Blender, grab the exact same version from 
   `python website <http://www.python.org>` and install it. 


#. Install MORSE using Homebrew::

    brew tap morse-simulator/morse
    brew install morse-simulator


.. note::

    If you have multiples version of Python3 installed in ``/usr/local/bin``,
    please specify the right one with ``--with-python=/usr/local/bin/python3.x.y`.


Formula Options
---------------

The default Homebrew formula configures and installs MORSE with support
for only the sockets middleware.  The following formula options are
available to enable support for other middlewares:

- ``--with-ros``
- ``--with-moos``
- ``--with-pocolibs``
- ``--with-yarp2``

Additionally documentation generating documentation and HLA support can
be enabled with the following flags:

- ``--with-doc``
- ``--with-hla``

Pymorse bindings can be enabled used the flag:

- ``--with-pymorse``

E.g. to install MORSE with generated documentation and support for ROS::

    brew install morse-simulator --with-doc --with-ros

Updating
--------

To update to the latest version of MORSE after installation::

    brew update
    brew install --upgrade morse-simulator

Uninstalling
------------

To uninstall MORSE::

    brew uninstall morse-simulator


ROS Installation Issues
-----------------------

ROS for OSX can be installed from source using Homebrew with the 
`instructions <http://www.ros.org/wiki/groovy/Installation/OSX/Homebrew/Source>`_.

In addition to the instructions provided on the ROS website, a Python 3
version of rospkg must be installed for MORSE to operate properly::

    sudo pip3 install rospkg

