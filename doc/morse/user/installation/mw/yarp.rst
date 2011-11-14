YARP 
~~~~

YARP can be installed either by hand, or with the aid of **robotpkg**.

Manually
++++++++

YARP has a number of requirements, and all of these packages must be installed
following the instructions they provide, in the following order:

- ACE ( 5.6.3 or +, required for YARP)
- SWIG (2.0.4, required to compile the Python bindings)
- YARP version (2.2.5 or +) (warning, there is a known issue with yarp-2.3.0,
  don't try to use MORSE with this version. The issue has been fixed with
  yarp-2.3.1).
- YARP python binding

Instructions to create YARP-Python bindings are `here
<http://eris.liralab.it/wiki/YARP_and_Python>`_.

Compiling the YARP Python binding will create two files: ``yarp.py`` and
``_yarp.so``. To be able to use the yarp module within Python, you must
copy said files to your Python lib directory
(``/usr/lib/python3.2/site-packages/``) or at some place reachable from your
``PYTHONPATH`` environment variable.

.. warning::
    The name of the installation directory may be different depending on your Linux distribution. If you use Ubuntu or similar distributions, replace the directory name of ``python3.2/site-packages`` for ``python3/dist-packages``. Make sure to indicate the correct path used in your computer for all Python 3 libraries.

.. note::
    To properly use simulated cameras with yarp < 2.3.2, you need to apply the patch from ``patches/yarp.i.diff``.


Packages manager
++++++++++++++++

Probably that the easiest way to install YARP is to use ``robotpkg`` (see
`robotpkg homepage <http://homepages.laas.fr/mallet/robotpkg>`_ for more
information). Follow the instructions on installing ``robotpkg``. Then add
the environment variable ``ROBOTPKG_BASE`` to your shell.  Then to install
YARP::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/yarp
  $ make update

Afterwards, install the ``morse`` specific bindings for Python::

  $ cd $ROBOTPKG_BASE/robotpkg/simulation/morse-yarp
  $ make update

This should take care of installing the required files ``yarp.py`` and ``_yarp.so``
to the correct location.
