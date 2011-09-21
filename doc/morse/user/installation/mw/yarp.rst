YARP 
~~~~

For the YARP bindings

- YARP version (2.2.5 or +) (warning, there is a known issue with yarp-2.3.0,
  don't try to use MORSE with this version. The issue has been fixed with
  yarp-2.3.1).
- YARP python binding
- ACE ( 5.6.3 or +, required for YARP)
- SWIG (2.0.4, required to compile the Python bindings)

Instructions to create YARP-Python bindings are `here
<http://eris.liralab.it/wiki/YARP_and_Python>`_.  To properly use simulated
cameras with yarp < 2.3.2, you need to apply the patch from
``patches/yarp.i.diff``.


Note that the easiest way to install YARP is probably to use ``robotpkg`` (see
`robotpkg homepage <http://homepages.laas.fr/mallet/robotpkg>`_ for more
informations). Follow the instructions on installing ``robotpkg``. Then add
the environment variable ``ROBOTPKG_BASE`` to your shell.  Then to install
``YARP`` ::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/yarp
  $ make update

Afterwards, add the following settings in ``${ROBOTPKG_BASE}/etc/robotpkg.conf`` ::

  $ echo "PKG_OPTIONS.py-yarp+= python3" >> ${ROBOTPKG_BASE}/etc/robotpkg.conf

and then install the YARP python bindings bindings ::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/py-yarp
  $ make update


Compiling the YARP Python binding will create two files: ``yarp.py`` and
``_yarp.so``, and install them in
``$ROBOTPKG_BASE/lib/python3.2/site-packages/`` You'll need to set the
environment variable ``PYTHONPATH`` to
``$ROBOTPKG_BASE/lib/python3.2/site-packages/`` to let python find the YARP
module.

If you are not using robotpkg to install YARP, then make sure to copy the
files ``yarp.py`` and ``_yarp.so`` to your Python lib directory
(``/usr/lib/python3.2/site-packages/``) or at some place reachable from your
``PYTHONPATH`` environment variable.

.. warning::
    The name of the installation directory may be different depending on your Linux distribution. If you use Ubuntu or similar distributions, replace the directory name of ``python3.2/site-packages`` for ``python3/dist-packages``. Make sure to indicate the correct path used in your computer for all Python 3 libraries.
