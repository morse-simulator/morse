MORSE installation
==================

MORSE in two minutes (if you run Debian/Ubuntu!)
------------------------------------------------

Fire a console, type ``sudo apt-get install morse-simulator`` (or click here:
`install morse-simulator <apt:morse-simulator>`_), then::

 $ morse create my_first_sim
 $ morse run my_first_sim

Here you are!

.. image:: ../../media/initial_sim.jpg
  :align: center

Note however that, **since Debian does not package ROS (or any other robotic
middleware), MORSE does not come with ROS support when installed this way!**

Read on to install support for your favorite middleware, or head to the
:doc:`Quickstart<../quickstart>` tutorial.

General pre-requisites
----------------------

Supported operating systems
+++++++++++++++++++++++++++

Only Linux (x86, x86_64) is currently officially supported. MORSE is mainly
developed on Fedora and Ubuntu, but we don't expect problems on other
distributions.

Other UNIXes systems probably work as well, like FreeBSD or Apple MacOSX.
Limited testing has been performed on OSX 10.8 with success (see
below for the Homebrew recipe).

MORSE does not currently officially support Microsoft Windows, although some
users reported success. Testers/maintainers for Windows are welcome!


Hardware
++++++++

A decent machine is required (typically, with an Intel i5 + 4GB RAM, you
should be comfortable).

To display textures correctly in the simulator, as well as to generate images
using the simulated cameras, you will need to have a graphics card that
supports GLSL shading. The Blender website lists these graphic cards as
compatible with GLSL:

- ATI Radeon 9x00, Xx00, X1x00, HD2x00 and HD3x00 series and newer.  
- NVidia Geforce FX, 6x00, 7x00, 8x00, 9x00 and GTX 2x0 and newer.

If you do not need cameras and OpenGL textures/shaders, you are advised to
run your simulation in ``fastmode`` (:doc:`refer to the simulation's Builder
API <../user/builder>`) for vastly improved loading time and performances.


Packaged versions
-----------------


MORSE is available on Debian Wheezy/Ubuntu >= 13.04. You can install
the package ``morse-simulator`` with your favorite software manager::

  $ sudo apt-get install morse-simulator

You can also install the Python bindings with::

  $ sudo apt-get install python3-morse-simulator

.. warning::
    Since the standard robotic middlewares (like ROS or Yarp) are **not** packaged
    in Debian, the Debian/Ubuntu ``morse-simulator`` package comes **without**
    support for robot middlewares!
    
    If you want to use MORSE with a robotic middleware, you **must** install it
    manually (see next section) or by using a robotic-specific package manager
    like ``robotpkg`` (see below).

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
- Python (3.3 or +)
- ``python-dev`` package
- Blender (>= 2.65) build with Python >= 3.3. You can simply get a binary from
  `Blender website <http://www.blender.org/download/get-blender/>`_


Middleware-specific instructions
++++++++++++++++++++++++++++++++

If you plan to use the simulator with raw sockets or text files as interface
(for instance, to use the :doc:`Python bindings  <../pymorse>` :tag:`pymorse` or
to use MORSE from MatLab or other specific applications), you do not need
to install anything specific, and you can jump to the next section.

Otherwise, check MORSE's installation notes for each of the desired middleware(s):

.. toctree::
    :glob:
    :maxdepth: 1

    installation/mw/*


- MORSE is also known to work with `OpenRTM
  <http://www.aisl.ics.tut.ac.jp/RTC/en/morse.html>`_.


Installation
++++++++++++


Clone with ``git`` or download the latest version of the source code::

  $ git clone https://github.com/morse-simulator/morse.git
  
(the lastest revision is always reasonably stable, and we recommend you to use it. However, if you prefer to use the stable branch, you can checkout the `1.2_STABLE` branch or download it `from here <https://github.com/morse-simulator/morse/releases/tag/1.2.2>`_.

MORSE relies on a standard `CMake` workflow: go to the directory where you
downloaded the MORSE source and type::

  $ mkdir build && cd build
  $ cmake ..


Several options (in particular to select the desired middlewares) can be passed
to ``cmake``. For instance, to build and install MORSE with ROS support, you
need something like::

  $ cmake -DBUILD_ROS_SUPPORT=ON -DCMAKE_BUILD_TYPE=Release ..

We recommend you to use ``ccmake ..`` to inspect (and modify) all the available
options. For instance, you may also want to set ``PYMORSE_SUPPORT`` to ``ON`` to install
the MORSE Python bindings.

Finally, compile with::

  $ sudo make install

The optional ``$MORSE_BLENDER`` environment variable can be set to let the
simulator know where to look for Blender if it is not accessible from the
path.

You can check your configuration is ok with::

  $ morse check

.. note::
    When updating MORSE to a more recent version, you'll simply have to do::

    $ git pull --rebase https://github.com/morse-simulator/morse.git master
    $ cd build
    $ sudo make install

Time to jump to MORSE's :doc:`Quickstart<../quickstart>` tutorial!


Advanced components
-------------------

If you want to distribute your simulation in a multinode infrastructure,
MORSE provides by default a socket service for multinode synchronization. If
you want to use HLA, you have to first install the CERTI and ``PyHLA`` packages:

.. toctree::
    :glob:
    :maxdepth: 1
    
    installation/mw/hla


Installation troubleshooting
----------------------------

In case of problems installing MORSE, verify the
:doc:`list of Frequently Asked Questions <faq>`.
