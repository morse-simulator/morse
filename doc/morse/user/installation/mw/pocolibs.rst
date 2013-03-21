Pocolibs
========

To build Pocolibs bindings (the LAAS-CNRS middleware), you need to install
Pocolibs on your system, and the different interface of needed modules.


The robotpkg way
----------------

The recommended way to do it is through ``robotpkg`` (see `robotpkg homepage
<http://homepages.laas.fr/mallet/robotpkg>`_ for more information).

To install::

  $ cd robotpkg/middleware/pocolibs
  $ make update

After that you will need to install the different modules you need to test
using MORSE.  This is done by going into the individual package directories
and running ``make update``.

Finally, you have to install the MORSE-Pocolibs bindings as well. To install
it, you simply need to::

  $ cd $ROBOTPKG_BASE/robotpkg/wip/morse-pocolibs
  $ make update

The support for video camera output (**viam**) and depth camera output
(**stereopixel**) is optional as it requires explicitly to the according
module. You can add this specific support by configuring options of package
morse-pocolibs (by editing ``${ROBOTPKG_BASE}/etc/robotpkg.conf`` or on the
command line)::

  PKG_OPTIONS.morse-pocolibs += viam

.. note::

	To make it working properly, you need to install all your genom modules
	with the robotpkg ``python`` option.
 
The manual way
--------------

If you want to build the interface for a genom module manually handled, do not
forget to pass the flag ``-y`` to genom so it generates the ``cstruct python
interface`` used by Morse to communicate with Pocolibs.

Specific configuration
----------------------

In addition to the normal configuration necessary to Morse, you need to care
about two other variables:

- genom does not install python interface in the standard ``PYTHONPATH``, so you
  need to add to your ``PYTHONPATH`` ``${PREFIX}/share/modules/python``.
- Morse relies on the environment variable ``LD_LIBRARY_PATH`` to localize
  ``libposterLib.so``, which is used to communicate with pocolibs. If pocolibs
  has been installed with robotpkg, it is located in ``${ROBOTPKG_BASE}/lib``.
  It is not recommended to set globally the variable in your shell, so it is
  better to use an alias in your shell configuration file::

	alias morse = 'env LD_LIBRARY_PATH=${ROBOTPKG_BASE}/lib morse'
