Running MORSE headless
======================

Running MORSE "headless" means running MORSE without launching a graphical
user interface (GUI) at all.

This is useful in several scenarios. For example, running MORSE on a remote server;
integrating MORSE in a Continuous Integration (CI) pipeline; running automated
tests, etc.

While "headless" really means "no GUI", it is also related to: "how
to run MORSE without 3D acceleration (i.e., without a GPU)". We address both
points below.

.. important::

  Headless MORSE is currently only tesed on Linux.

Overview
--------

It is important to understand that MORSE *requires OpenGL*. There is
currently no way around this, and, as a 3D simulator, we are likely to keep this
requirement for the foreseeable future.  So, running a "headless" MORSE still
requires that your OS provides an OpenGL implementation.

OpenGL *does not* mandate a GPU, though. So, it is perfectly possible to run
MORSE on a CPU-only computer (on a server, on a cluster in the cloud without
GPUs, etc). Obviously, no GPU means no 3D hardware acceleration, but depending
on your application, performance may still be perfectly okay. Some non-scientific
benchmarks are provided below, for reference.

To run "headless", we also need to prevent MORSE's main window from appearing. This
can easily be achieved by using `Xvfb <https://en.wikipedia.org/wiki/Xvfb>`_ a
special graphic server that renders 3D applications to memory instead of to the
screen.

.. important::

  The Xvfb X server does not support *hardware* acceleration, and will not make
  use of your GPU, even if you have one. Alternatives like `Xdummy
  <http://xpra.org/trac/wiki/Xdummy>`_ exist, but have not been tested.

CPU-based OpenGL
----------------

In the Linux world, the best option for CPU-based 3D acceleration is `LLVMpipe
<http://www.mesa3d.org/llvmpipe.html>`_. It is fairly easy to compile and
install.

- Install the system dependencies. For Debian/Ubuntu, ``apt-get install llvm-dev
  scons python-mako libedit-dev``
- Grab the lastest version of Mesa here (tested with 9.2.2 and 11.0.7): `mesa
  FTP <ftp://ftp.freedesktop.org/pub/mesa/>`_.
- Compile with:

::

    $ scons build=release llvm=yes libgl-xlib

This will result in a new ``libGL.so`` that uses the CPU instead of the
GPU. Blender runs pretty well with it.

To run MORSE with this library:

::

    LD_LIBRARY_PATH=<path to your Mesa>/build/linux-x86_64/gallium/targets/libgl-xlib morse run <env>

Some quick performance results with the default environment:

::

    $ morse create test

Then, edit ``test/default.py`` and add ``env.show_framerate()`` at the end
to display the FPS.

On a test computer
with `mesa-11.0.7` and LLVMpipe on an Intel Core i7-4790 @ 3.6GHz and 16GB RAM,
we achieved 30 FPS vs 60 FPS with 3D acceleration.

By only rendering wireframes (i.e., using MORSE's *fast mode*: edit ``test/default.py``
and switch ``fastmode`` to ``True``), we reached 60 FPS.

.. note::

  In fast mode, you only get the wireframe of the models: this is fine if
  you do not do any vision-based sensing.


Going headless
--------------

To prevent MORSE from opening a window, you can use ``Xvfb`` to create
a 'fake' display to run MORSE. On debian/Ubuntu, ``apt-get install xvfb``.

::

    $ Xvfb -screen 0 1024x768x24 :1 &
    $ LD_LIBRARY_PATH=<LLVMpipe path> DISPLAY=:1 morse <...>

Which such a configuration, we successfully streamed full 3D 800x600 images with
ROS at 20Hz.

.. note::

    While you need to create a display with a 24-bit color depth,
    the size can be made smaller, for further improved performances. For
    instance, 100x100px is big enough to run MORSE. However, if you
    use video cameras, note that you can't stream images larger than the window size.


.. note::

  Assuming ``imagemagik`` is installed, you can easily take a screenshot of your
  invisible window with ``DISPLAY=:1 import -window root screenshot.png``


