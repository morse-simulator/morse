Pocolibs
~~~~~~~~

To build Pocolibs bindings (the LAAS-CNRS middleware), you need to install Pocolibs on your system.

The recommended way to do it is through ``robotpkg`` (see `robotpkg homepage
<http://homepages.laas.fr/mallet/robotpkg>`_ for more informations).

To install::

  $ cd $ROBOTPKG_BASE/robotpkg/middleware/pocolibs
  $ make update


After that you will need to install the different modules you need to test using MORSE.
This is done by going into the individual package directories and running ``make update``.

Finally, you have to install the MORSE-Pocolibs bindings as well.
At the time of writing, there are two options for installing,
depending on the type of simulation needed, and they affect which modules will be linked:

* **hri** (for human-robot interaction) 
* **outdoor**

To specify the type of simulation (**outdoor** in this example),
it is necessary to edit the file
``$ROBOTPKG_BASE/etc/robotpkg.conf`` and add the line::

  PKG_OPTIONS.morse-pocolibs += outdoor

After that, install the module with these instructions::

  $ cd $ROBOTPKG_BASE/robotpkg/wip/morse-pocolibs
  $ make update
