MORSE installation
==================

Requirements - What you need to install before 
----------------------------------------------

Hardware
++++++++

To display textures correctly in the simulator, as well as to generate images
using the simulated cameras, you will need to have a graphics card that
supports GLSL shading. The Blender website lists these graphic cards as
compatible with GLSL:

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

- Python (3.2 or +) compiled with the ``--with-wide-unicode`` flag
- Blender (>= 2.59 & < 2.62) build with Python 3.2
- MORSE source code

.. note::
  If you install Python by hand, it is important to specify the
  ``--with-wide-unicode`` flag, since Blender expects this behaviour.
  Otherwise, there will be an incompatibility of types when using additional
  middlewares.
 
If you plan to use the simulator with raw sockets of text files as "middleware",
you don't need anything else. Otherwise, you need to install the software for
other middlewares:

.. toctree::
    :glob:
    :maxdepth: 1

    installation/mw/*
    
If you want to distribute your simulation in a multinode infrastructure, MORSE
provides by default a socket service for multinode synchronization. If you
want to use HLA, you have to first install the CERTI and PyHLA packages:

.. toctree::
    :glob:
    :maxdepth: 1
    
    installation/hla


Installation 
------------

.. note::
    The directory where MORSE is installed will be referred to as ``$MORSE_ROOT`` in this document.

It is recommended to store this environment variable, as it is necessary to
use the :doc:`Builder API scripts <../dev/builder>` to generate simulation
scenes with custom equipped robots.

Manually
++++++++

Download the latest version of the source code. It is stored in a ``git``
repository::

  $ git clone http://github.com/laas/morse.git
  
If you want to get only the latest stable version (0.5) of Morse, you can get
it in the branch `0.5_STABLE`. You can get it directly using ::
 
  $ git clone http://github.com/laas/morse.git -b 0.5_STABLE

or if you have already download the repository ::
  
  $ git checkout -b 0.5_STABLE -t origin/0.5_STABLE
  
You can get a `tarball version here <https://github.com/laas/morse/tarball/0.5>`_. 

 
Go to the directory where you have previously downloaded the MORSE source.
Then type these commands::

  $ mkdir build && cd build
  $ cmake ..

By default, MORSE will install in ``/usr/local``. You can easily change the
install directory by giving additional parameters to ``cmake``.
You can also change the installation type and select the middleware bindings
by using these additional parameters.

- ``CMAKE_INSTALL_PREFIX`` controls where will be installed MORSE. The install
  prefix directory is referred to as ``$MORSE_ROOT``.
- ``BUILD_CORE_SUPPORT`` controls the builds and install of Morse core. It is
  ON by default
- ``BUILD_DOC_SUPPORT`` controls the build of the documentation (require
  sphinx)
- ``BUILD_HLA_SUPPORT`` controls the builds of HLA support for multi-node
  simulations in MORSE.
- ``BUILD_POCOLIBS_SUPPORT`` controls the build of pocolibs support in MORSE.
- ``BUILD_YARP2_SUPPORT`` controls the build of YARP support in MORSE.
- ``BUILD_ROS_SUPPORT`` controls the build of ROS support in MORSE.
- ``BUILD_MOOS_SUPPORT`` controls the build of MOOS support in MORSE.
- ``PYMORSE_SUPPORT`` controls the build and installation of pymorse, a
  library to interact with Morse through the socket interface. It is needed
  for test infrastructure.
- ``CMAKE_BUILD_TYPE`` controls the optimization stuff for C/C++ extension
  (Release is a good choice).
- ``PYTHON3_EXECUTABLE`` indicate where the python3 executable is in your system
- ``PYTHON3_INCLUDE_DIR`` indicate the directory where Python headers are installed
- ``PYTHON3_LIBRARY`` indicate the directory where Python libraries are installed

You can set up the different variables using the command line.
For instance, to build and install MORSE with YARP support in ``/opt``, you need something like::

  $ cmake -DBUILD_YARP2_SUPPORT=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt ..

Or to tell MORSE where to find a Python installed in a different location::

  $ cmake -DPYTHON3_INCLUDE_PATH=/usr/local/include/python3.2mu -DPYTHON3_LIBRARY=/usr/local/lib/python3.2mu.a ..

Alternatively, you can use ``ccmake ..`` to change all of these parameters using a
graphical interface. You can modify many different variables by switching to
"advanced mode" (pressing the ``t`` key).

After configuring the necessary parameters, compile with::

  $ sudo make install

The optional ``$MORSE_BLENDER`` environment variable can be set to let the
simulator know where to look for Blender if it is not accessible from the
path.

You can check your configuration is ok with::

  $ morse check

.. note::
    When updating MORSE to a more recent version, you'll simply have to do::

    $ git checkout [version]
    $ cd build
    $ make install


Packages manager
++++++++++++++++

MORSE is available through some package manager. See their associated
documentation.

.. toctree::
    :glob:
    :maxdepth: 1

    installation/package_manager/*


Installation troubleshooting
----------------------------

In case of problems installing MORSE, verify the
:doc:`list of Frequently Asked Questions <faq>`.
