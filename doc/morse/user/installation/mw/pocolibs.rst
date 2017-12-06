Pocolibs
========

To build Pocolibs bindings (the LAAS-CNRS middleware), you need to install
Pocolibs on your system, as well as the interfaces of the modules you
want to use.


The robotpkg way
----------------

The recommended way to do it is through ``robotpkg`` (see `robotpkg homepage
<http://robotpkg.openrobots.org>`_ for more information).

To install::

  $ cd robotpkg/middleware/pocolibs
  $ make update

After this, you will need to install the different modules you need to test
using MORSE.  This is done by going into the individual package directories
and running ``make update``.

Finally, you must also install the MORSE-Pocolibs bindings. To install
these, you simply need to do this::

  $ cd $ROBOTPKG_BASE/robotpkg/wip/morse-pocolibs
  $ make update

The support for video camera output (**viam**) and depth camera output
(**stereopixel**) is optional since their availability depends on the
module being used.
You can add this specific support by configuring the options for the
morse-pocolibs package (by editing ``${ROBOTPKG_BASE}/etc/robotpkg.conf`` or on the
command line)::

  PKG_OPTIONS.morse-pocolibs += viam

.. note::

	To make it working properly, you need to install all your genom modules
	with the robotpkg ``python`` option.
 
The manual way
--------------

If you want to build the interface for a genom module by hand, remember
to pass the flag ``-y`` to genom so that it generates the ``cstruct python
interface`` used by Morse to communicate with Pocolibs.

Specific configuration
----------------------

In addition to the normal configuration necessary for Morse, you must
take care of two variables:

- genom does not install its Python interface in the standard ``PYTHONPATH``, so you
  need to add to your ``PYTHONPATH``, ``${PREFIX}/share/modules/python``.
- Morse relies on the environment variable ``LD_LIBRARY_PATH`` to find
  ``libposterLib.so``, which is used to communicate with pocolibs. If pocolibs
  has been installed with robotpkg, it is located in ``${ROBOTPKG_BASE}/lib``.
  It is not recommended to globally set this variable in your shell
  (since it may affect other software), so it is
  better to use an alias in your shell configuration file::

	alias morse = 'env LD_LIBRARY_PATH=${ROBOTPKG_BASE}/lib morse'
