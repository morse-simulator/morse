MORSE installation
==================

MORSE in two minutes (if you run Debian or Ubuntu!)
------------------------------------------------

Open a console, type ``sudo apt-get install morse-simulator`` (or click here:
`install morse-simulator <apt:morse-simulator>`_), then::

 $ morse create my_first_sim
 $ morse run my_first_sim

This will start running a simple simulation straight away!

.. image:: ../../media/environments/sandbox.jpg
  :align: center

Installing the morse simulation will also ensure that you have full support for :tag:`ros`,
:tag:`yarp`, :tag:`moos`, and in fact, :doc:`all of the middlewares supported by
MORSE <../user/integration>` (with a few corner cases for the :tag:`pocolibs`
middleware that may require manual compilation. Read on).

.. note::
  Automatic installation of middlewares' support requires MORSE >=1.3! **Ubuntu =<
  15.04 and Debian Wheezy/Jessie only ship MORSE 1.2.2**: in that case, you
  still need to install MORSE manually if you want to use MORSE with
  ROS/YARP/MOOS/pocolibs. Read on.

If you've successfully installed the simulator, you can now dive into
the :doc:`Quickstart<../quickstart>` tutorial!

System requirements
-------------------

Supported operating systems
+++++++++++++++++++++++++++

Only Linux (``x86``, ``x86_64``) is currently officially supported. MORSE is
mainly developed on Fedora and Ubuntu, but it should work fine on other Linux
distributions.

It should also work on UNIX systems more generally, such as FreeBSD and Apple MacOSX.
Some successful testing has been done on OSX 10.8 (see
below for the Homebrew recipe).

Windows is supported, but not fully tested.


Hardware
++++++++

A decent machine is required (for example, an Intel i5 with 4GB RAM
should be sufficient).

To display textures correctly in the simulator, as well as to generate images
using the simulated cameras, you will need to have a graphics card that
supports GLSL shading. The Blender website lists the following
GLSL-compatible graphics cards:

- ATI Radeon 9x00, Xx00, X1x00, HD2x00 and HD3x00 series and newer.  
- NVidia Geforce FX, 6x00, 7x00, 8x00, 9x00 and GTX 2x0 and newer.

If you don't need cameras and OpenGL textures/shaders, you are advised to
run your simulation in ``fastmode`` (:doc:`refer to the simulation's Builder
API <../user/builder>`) for vastly improved load time and runtime performance.


Packaged versions
-----------------

MORSE is available on Debian >= Wheezy and Ubuntu >= 13.04. You can install
the package ``morse-simulator`` with your preferred software package manager::


  $ sudo apt-get install morse-simulator

You can also install the Python bindings with::

  $ sudo apt-get install python3-morse-simulator

.. warning::

  Automatic installation of middlewares' support requires MORSE >=1.3! **Ubuntu =<
  15.04 and Debian Wheezy/Jessie only ship MORSE 1.2.2**: in that case, you
  need to install MORSE manually if you want to use MORSE with
  ROS/YARP/MOOS/pocolibs. Continue to the next section.


You can also easily install MORSE with:

.. toctree::
    :glob:
    :maxdepth: 1

    installation/package_manager/*

See their associated documentation for details.


Manual installation
-------------------

.. note::
    The directory where MORSE is installed will be referred to as ``$MORSE_ROOT`` in this document.

Prerequisites
+++++++++++++

- ``cmake``
- Python (>= 3.3)
- ``python-dev`` package
- Blender (>= 2.65) built with Python >= 3.3. You can simply get a binary from
  `Blender website <http://www.blender.org/download/get-blender/>`_
- numpy for Python 3 (often ``py3-numpy `` or ``python3-numpy``)


.. note::
    If you plan to build packages for MORSE, please read the notes on packaging below.

.. note::
    If you use a Blender binary, numpy is included within it.

.. note::
    If building on Windows, ensure that your Python version and architecture matches
    the bundled Python in Blender (currently Python 3.5.3 for Blender 2.79).

Installation
++++++++++++

Linux
-----

Clone with ``git`` or download the latest version of the source code::

  $ git clone https://github.com/morse-simulator/morse.git
  
The latest revision is always reasonably stable, and we recommend that you use
it. However, if you prefer to use the stable branch, you can checkout the
``1.4_STABLE`` branch or download it `from here
<https://github.com/morse-simulator/morse/releases/latest>`_.

MORSE relies on a standard `CMake` workflow: go to the directory where you
downloaded the MORSE source and type::

  $ mkdir build && cd build
  $ cmake ..
  $ sudo make install

While the default set of options should cover most users needs, you can
use ``ccmake ..`` to inspect (and modify) all the available options.

The optional ``$MORSE_BLENDER`` environment variable can be set to tell the
simulator where to look for Blender if it is not accessible from the path.

You can check your configuration is okay with::

  $ morse check

.. note::
    When updating MORSE to a more recent version, you only have to do::

    $ git pull --rebase https://github.com/morse-simulator/morse.git master
    $ cd build
    $ sudo make install

Once MORSE is successfully installed and checked you are read for the
:doc:`Quickstart<../quickstart>` tutorial!

Windows
-------

Download the latest version of the source code. It is stored in a git
repository::

  $ git clone https://github.com/morse-simulator/morse.git

The MORSE_BLENDER environment variable should be set the the location and filename
of the Blender executable (ie "C:\Program Files\Blender\blender.exe").

Additionally, both cmake and Python should be on the system path.

Go to the directory where you have previously downloaded the MORSE source.
Then run the winbuild.bat script.

By default, MORSE will install in C:\morse. You can easily change the
install directory by editing the CMAKE_INSTALL_PREFIX variable in
winbuild.bat

Middleware-specific notes
+++++++++++++++++++++++++

- :tag:`ros` Since MORSE runs with Python 3, you need to install the packages
  ``python3-catkin-tools`` and ``python3-yaml``.  If these
  packages are not available for your distribution, :doc:`check the manual
  instructions <installation/mw/ros>`.
- :tag:`yarp` If needed, instructions on how to install YARP on your system are :doc:`available here <installation/mw/yarp>`.
- :tag:`moos` If needed, instructions on how to install MOOS on your system are :doc:`available here <installation/mw/moos>`.
- :tag:`pocolibs` ``pocolibs`` support requires specific steps (some
  bindings require a compilation). The instructions are :doc:`available here <installation/mw/pocolibs>`.



.. note::
    While not officially supported, MORSE is also known to work with `OpenRTM
    <http://www.aisl.ics.tut.ac.jp/RTC/en/morse.html>`_.



Advanced components
-------------------

If you want to distribute your simulation in a multinode infrastructure,
MORSE provides by default a socket service for multinode synchronization. If
you want to use HLA, you must first install the CERTI and ``PyHLA`` packages:

.. toctree::
    :glob:
    :maxdepth: 1
    
    installation/mw/hla

Notes for packaging
-------------------


By default, MORSE automatically installs support for every supported middleware
as well as the Python bindings ``pymorse``.

However, when you package MORSE yourself, you may want to separate the
support for the various middlewares from the core of the simulator. This
can be easily achieved by passing options to `CMake` like
``-DBUILD_ROS_SUPPORT=OFF`` or ``PYMORSE_SUPPORT=OFF``.

``-DBUILD_CORE_SUPPORT=OFF`` disables the installation of the
simulator's core. Doing this allows you to separately package the
support for the various middlewares/bindings.

Installation troubleshooting
----------------------------

In case of problems installing or running MORSE, check the
:doc:`list of Frequently Asked Questions <faq>`.
