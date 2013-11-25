With Homebrew on OSX
++++++++++++++++++++

`Homebrew <http://brew.sh>`_ is a package manager for OSX.
It can be installed in OSX with a single Ruby command::

    ruby -e "$(curl -fsSL https://raw.github.com/mxcl/homebrew/go)"


.. Note::
    Homebrew requires XCode and the command line tools to be installed.

Installing MORSE
----------------

#. Download `Blender <http://www.blender.org/download/get-blender/>`_ and
   copy the apps to ``/Applications``. Blender is not currently available
   as a recipe in Homebrew.

   .. Note::
        ``MORSE_BLENDER`` should point to the actual executable, not the app
        directory::

            MORSE_BLENDER=/Applications/Blender.app/Contents/MacOS/blender

#. Install Python3 using Homebrew::

    brew install python3

#. Install MORSE using Homebrew::

    brew tap morse-simulator/morse
    brew install morse-simulator

.. Note::
    The Python3 version installed must exactly match the Python version
    used by Blender.  The latest Homebrew Python3 formula installs
    Python 3.3.1 while the latest Blender version uses 3.3.0.  To install
    3.3.0 run the following commands before installing python3::

        cd /usr/local
        git checkout 864e9f1 /usr/local/Library/Formula/python3.rb

    The provided hash can be found by running::

        brew versions python3


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

