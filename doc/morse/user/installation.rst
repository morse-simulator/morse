MORSE installation 
==================

Requirements - What you need to install before 
----------------------------------------------

Hardware
++++++++

To display textures correctly in the simulator, as well as to generate images using the simulated cameras, you will need to have a graphics card that supports GLSL shading. The Blender website lists these graphic cars as compatible with GLSL:

- ATI Radeon 9x00, Xx00, X1x00, HD2x00 and HD3x00 series and newer.
- NVidia Geforce FX, 6x00, 7x00, 8x00, 9x00 and GTX 2x0 and newer.

Supported operating systems
+++++++++++++++++++++++++++

Only Linux (x86, x86_64) is currently officially supported. MORSE is mainly
developed on Fedora and Ubuntu, but we don't expect problems on other
distributions.

Other UNIXes systems probably work as well (like FreeBSD or Apple MacOSX).

MORSE does not currently support Microsoft Windows, although it may work
(testers/maintainers for Windows are welcome!).

Required software
+++++++++++++++++

.. note::
  If you use the automated ``robotpkg``-based installation (recommended), you can skip this section: 
  ``robotpkg`` will check and install for you all required dependencies.

- Python (3.1 or +)
- Blender 2.54+ build with Python 3.1
- MORSE source code
 
If you plan to use the simulator with raw sockets of text files as "middleware",
you don't need anything else. Otherwise, you need to install the software for other middlewares.

YARP 
~~~~

For the YARP bindings

- YARP version (2.2.5 or +) (warning, there is a known issue with yarp-2.3.0, don't try to use Morse with this version. The issue has been fixed with yarp-2.3.1).
- YARP python binding
- ACE ( 5.6.3 or +, required for YARP)

Instructions to create YARP-Python bindings are `here <http://eris.liralab.it/wiki/YARP_and_Python>`_.
To properly use simulated cameras with yarp < 2.3.2, you need to apply the patch from ``patches/yarp.i.diff``.


Note that the easiest way to install YARP is probably to use ``robotpkg`` (see `robotpkg homepage <http://homepages.laas.fr/mallet/robotpkg>`_ for more informations). Follow the instructions on installing ``robotpkg``. Then add the environment variable ``ROBOTPKG_BASE`` to your shell.
Then to install ``YARP`` ::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/yarp
  $ make update

Afterwards, add the following settings in ``${ROBOTPKG_BASE}/etc/robotpkg.conf`` ::

  $ echo "PKG_OPTIONS.libpyyarp+= python3" >> ${ROBOTPKG_BASE}/etc/robotpkg.conf

and then install the YARP python bindings bindings ::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/libpyyarp
  $ make update


Compiling the YARP Python binding will create two files: ``yarp.py`` and ``_yarp.so``, and install them in ``$ROBOTPKG_BASE/lib/python3.1/site-packages/``
You'll need to set the environment variable ``PYTHONPATH`` to ``$ROBOTPKG_BASE/lib/python3.1/site-packages/`` to let python find the YARP module.

If you are not using robotpkg to install YARP, then make sure to copy the files ``yarp.py`` and ``_yarp.so`` to your Python lib directory (``/usr/lib/python3.1/site-packages/``) or at some place reachable from your ``PYTHONPATH`` environment variable.

.. warning::
    The name of the installation directory may be different depending on your Linux distribution. If you use Ubuntu or similar distributions, replace the directory name of ``python3.1/site-packages`` for ``python3/dist-packages``. Make sure to indicate the correct path used in your computer for all Python 3 libraries.

ROS 
~~~

Blender 2.5x relies on Python3.x which is currently (April 2011) not supported by ROS.

The following steps explains how to get a working Python3 ROS setup, suitable for use with MORSE.

#. Install ROS Diamondback (check http://www.ros.org/wiki/ROS/Installation if needed)
#. Install Python3.x (these instructions were tested with Python3.1) manually
   or using your system package manager and make sure, your Pythonpath variable
   is pointing to the Python3-libraries
#. Install PyYAML with Python3 support (PyYAML >= 3.09, you can get it from http://pyyaml.org/)
   Install it with ``python3.1 setup.py install`` to be sure to have the Python3 libraries
#. Create a Python3-compatible overlay of your ROS installation using rosinstall with our Python3-rosinstall-file:
 
   ``rosinstall ~/ros-py3 /opt/ros/diamondback http://ias.cs.tum.edu/~kargm/ros_py3.rosinstall``
   (if your ROS is installed in /opt/ros/diamondback and your overlay should be created in ~/ros-py3)
   The ROS-stacks ros, ros_comm and common_msgs are overlayed by Python3-compatible versions and need to be rebuild:
   ``rosmake ros && rosmake ros_comm && rosmake common_msgs``

   Note: Rebuilding the common_msgs stack allows you to use all messages in
   this stack for communicating between MORSE and ROS. If you want to use any
   other messages, make sure the source-files are Python2 AND Python3
   compatible!


Pocolibs
~~~~~~~~

To build Pocolibs bindings (the LAAS-CNRS middleware), you need to install Pocolibs on your system.

The recommended way to do it is through ``robotpkg`` (see `robotpkg homepage <http://homepages.laas.fr/mallet/robotpkg>`_ for more informations).

To install::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/pocolibs
  $ make update

MOOS
~~~~~~~~

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

Installation 
------------

.. note::
    The directory where MORSE is installed will be referred to as ``$MORSE_ROOT`` in this document.

With ``robotpkg``
+++++++++++++++++

``robotpkg`` is a package manager for robotic software based on NetBSD ports.
It supports Linux, * BSD and Darwin (MacOS X).

.. Note::
	If you are upgrading an previous morse installation, skip directly to step 2.

#. Install and bootstrap ``robotpkg`` and ``robotpkg-wip`` using these
   instructions: `robotpkg installation <http://robotpkg.openrobots.org>`_ and 
   `robotpkg-wip installation <http://homepages.laas.fr/mallet/robotpkg-wip>`_
   (should take less than 5 min)
#. Add the following environment variables to your system::
    
    # If using tcsh
    setenv ROBOTPKG_BASE $HOME/openrobots
    setenv PKG_CONFIG_PATH $HOME/openrobots/lib/pkgconfig

    # If using bash
    export ROBOTPKG_BASE=$HOME/openrobots
    export PKG_CONFIG_PATH=$HOME/openrobots/lib/pkgconfig

#. Go to ``$ROBOTPKG/simulation/morse``
#. Type ``make update``
#. Go have a coffee :-) ``robotpkg`` will download and compile for you all the
   required dependencies, including Blender.
#. The previous package only installs middleware support for text and socket.
   If you want support for additional middlewares, repeat the operation in
   ``$ROBOTPKG/simulation/morse-yarp``, ``$ROBOTPKG/wip/morse-pocolibs``.

By hand
+++++++

Download the source code. It is stored in a ``git`` repository::

  $ git clone http://trac.laas.fr/git/robots/morse.git
  
Alternatively, you can use the GitHub mirror (synchronized every hour, probably a lot faster) ::
  
  $ git clone http://github.com/laas/morse.git
  
Once you have a copy of the repository, you can get to the last stable
version (0.3) by using ::
  
  $ git checkout 0.3
  
You can get a `tarball version here <https://github.com/laas/morse/tarball/0.3>`_. 

 
Go to the directory where you have previously downloaded the MORSE source. Then type these commands::

  $ mkdir build && cd build
  $ cmake ..

By default, MORSE will install in ``/usr/local``. You can easily change that by launching ``ccmake`` instead of ``cmake``.
When using ``ccmake``, it is also possible to select the optional middleware bindings for YARP and Pocolibs.

- ``CMAKE_INSTALL_PREFIX`` controls where will be installed MORSE. The install prefix directory is referred to as ``$MORSE_ROOT``.
- ``BUILD_POCOLIBS_SUPPORT`` controls the build of pocolibs support in MORSE
- ``BUILD_YARP2_SUPPORT`` controls the build of YARP support in MORSE
- ``CMAKE_BUILD_TYPE`` controls the optimization stuff for C/C++ extension (Release is a good choice). ::

  $ sudo make install

You can set up the different variables using the command line.
For instance, to build and install MORSE with YARP support in ``/opt``, you need something like::

  $ cmake -DBUILD_YARP2_SUPPORT=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt ..

The optional ``$MORSE_BLENDER`` environment variable can be set to let the simulator know where to look for Blender if it is not accessible from the path.

You can check your configuration is ok with::

  $ morse check

.. note::
    When updating MORSE to a more recent version, you'll simply have to do::

    $ git checkout [version]
    $ cd build
    $ make install


Running a simulation 
--------------------

[YARP specific] Before starting a simulation: Start the YARP's server using this command in a separate terminal::

  $ yarp server

Launch MORSE by calling the executable::

  $ morse

Several options are available, check them with::

  $ morse help

Once launched, you can test the simulator by loading one of the example scenarii from ``$MORSE_ROOT/share/examples/morse/scenarii`` (.blend files).

To start a simulation, go on Blender and press :kbd:`P` to play the scenario.

Tips: If you have any problem to start to play a simulation: start ``blender``
from a terminal and send the error messages to <morse_dev@laas.fr>.
Note that certain scenario files are configured to use various middlewares, and will need the middleware manager to be started beforehand.

Testing
-------

To test the external control clients:

- On a text terminal, run the ``morse`` command
- Open the Blender file: ``$MORSE_ROOT/share/examples/morse/tutorials/tutorial-1-solved.blend``
- Start the simulation :kbd:`P`
- On a separate terminal, go to the root directory of the MORSE source code
- Run the Python program::

  $ python examples/morse/clients/atrv/socket_v_omega_client.py

- Follow the client program's instructions to send movement commands to the robot and to read information back
- To finish the simulation, press :kbd:`esc`
- To close Blender, press :kbd:`C-q`, and then :kbd:`enter`
