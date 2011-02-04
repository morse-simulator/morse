MORSE installation 
==================

Requirements - What you need to install before 
----------------------------------------------

Hardware
++++++++

To display textures correctly in the simulator, as well as to generate images using the simulated cameras, you will need to have a graphics card that supports GLSL shading. The Blender website lists these graphic cars as compatible with GLSL:

- ATI Radeon 9x00, Xx00, X1x00, HD2x00 and HD3x00 series and newer.
- NVidia Geforce FX, 6x00, 7x00, 8x00, 9x00 and GTX 2x0 and newer.

Required software
+++++++++++++++++

.. note::
  If you want the automated ``robotpkg``-based installation (recommanded), you can skip this section: 
  ``robotpkg`` will check and install for you all required dependencies.

- Python (3.1 or +)
- Blender 2.54+ build with Python 3.1
- git to get the code of the simulator::

    $ git clone http://trac.laas.fr/git/robots/morse.git
  
  Alternatively, you can use the GitHub mirror (synchronized every hour, probably a lot faster) ::
  
    $ git clone http://github.com/laas/morse.git
  
  Once you have a copy of the repository, you can get to the last stable version (0.2b2) by using ::
  
    $ git checkout 0.2
  
  You can get a `tarball version here <https://github.com/laas/morse/tarball/0.2>`_. 
  
..  You can check the following information to make sure that the download went fine. ::
  
    SHA1 (morse-0.2b2.tar.gz) = 4ccdc81949282eda88121af04cf96b27696167f7
    RMD160 (morse-0.2b2.tar.gz) = f3867347a2c4beac43f895e3c51a509b4cdb5b7a
    Size (morse-0.2b2.tar.gz) = 36737879 bytes

If you plan to use the simulator with raw sockets of text files as "middleware",
you don't need anything else. Otherwise, you need to install the software for other middlewares.

YARP 
~~~~

For the YARP bindings

- YARP version (2.2.5 or +) (warning, there is a known issue with yarp-2.3.0, don't try to use Morse with this version. The issue has been fixed with yarp-2.3.1).
- YARP python binding
- ACE ( 5.6.3 or +, required for YARP)

Instructions to create YARP-Python bindings are `here <http://eris.liralab.it/wiki/YARP_and_Python>`_.
To use properly camera with yarp < 2.3.2, you need to apply the patch from patches/yarp.i.diff.


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

NOTE: The name of the installation directory may be different depending on your distribution. If you use Ubuntu or similar distributions, replace the directory name of ``site-packages`` for ``dist-packages``.

Pocolibs
~~~~~~~~

To build Pocolibs bindings (the LAAS-CNRS middleware), you need to install Pocolibs on your system.

The recommended way to do it is through ``robotpkg`` (see `robotpkg homepage <http://homepages.laas.fr/mallet/robotpkg>`_ for more informations).

To install::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/pocolibs
  $ make update

Installation 
------------

With ``robotpkg``
+++++++++++++++++

``robotpkg`` is a package manager for robotic software based on NetBSD ports. It supports Linux, * BSD and Darwin (MacOS X).

#. Install and bootstrap ``robotpkg`` and ``robotpkg-wip`` using these
   instructions: `robotpkg installation <http://robotpkg.openrobots.org>`_ (should
   take less than 5 min)
#. Go to ``$ROBOTPKG/wip/morse``
#. Configure the package settings PKG_OPTIONS.morse in ``${ROBOTPKG_BASE}/etc/robotpkg.conf``
#. Type ``make update``
#. Go have a coffee :-) ``robotpkg`` will download and compile for you all the required dependencies, including Blender.

By hand
+++++++

From your MORSE root directory::

  $ mkdir build && cd build
  $ cmake ..

By default, MORSE will install in ``/usr/local``. You can easily change that by launching ccmake instead of cmake.
When using ccmake, it is also possible to select the optional middleware bindings for YARP and Pocolibs.
You can set up the different variables using the command line:

- ``CMAKE_INSTALL_PREFIX`` controls where will be installed MORSE. Note: The install prefix directory will be referred to as ``$MORSE_ROOT`` in this document.
- ``BUILD_POCOLIBS_SUPPORT`` controls the build of pocolibs support in MORSE
- ``BUILD_YARP2_SUPPORT`` controls the build of YARP support in MORSE
- ``CMAKE_BUILD_TYPE`` controls the optimization stuff for C/C++ extension (Release is a good choice). ::

  $ sudo make install

For instance, to build and install MORSE with YARP support in ``/opt``, you need something like::

  $ cmake -DBUILD_YARP2_SUPPORT=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt ..

The optional ``$MORSE_BLENDER`` environment variable can be set to let the simulator know where to look for Blender if it is not accessible from the path.

You can check your configuration is ok with::

  $ morse check
  
Running a simulation 
--------------------

[YARP specific] Before starting a simulation: Start the YARP's server using this command in a separate terminal::

  $ yarp server

Launch MORSE by calling the morse executable::

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
- On a separate terminal, go to the root directory the MORSE source code
- Run the Python program::

  $ python examples/morse/clients/atrv/socket_v_omega_client.py

- Follow the client program's instructions to send movement commands to the robot and to read information back
- To finish the simulation, press :kbd:`esc`
- To close Blender, press :kbd:`C-q`, and then :kbd:`enter`
