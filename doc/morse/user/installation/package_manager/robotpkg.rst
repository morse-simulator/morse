With ``robotpkg``
+++++++++++++++++

``robotpkg`` is a package manager for robotic software based on NetBSD ports.
It supports Linux, * BSD and Darwin (MacOS X).

.. Note::
	If you are upgrading an previous morse installation, skip directly to step 2.

#. Install and bootstrap ``robotpkg`` and ``robotpkg-wip`` using these
   instructions: `robotpkg installation <http://robotpkg.openrobots.org>`_ and 
   `robotpkg-wip installation <http://homepages.laas.fr/mallet/robotpkg-wip>`_
   (should take less than 5 min)
#. Add the following environment variables to your system::
    
    # If using tcsh
    setenv ROBOTPKG_BASE $HOME/openrobots
    setenv PKG_CONFIG_PATH $HOME/openrobots/lib/pkgconfig

    # If using bash
    export ROBOTPKG_BASE=$HOME/openrobots
    export PKG_CONFIG_PATH=$HOME/openrobots/lib/pkgconfig

#. Go to ``$ROBOTPKG/simulation/morse``
#. Type ``make update``
#. Go have a coffee :-) ``robotpkg`` will download and compile for you all the
   required dependencies, including Blender.
#. The previous package only installs middleware support for text and socket.
   If you want support for additional middlewares, repeat the operation in
   ``$ROBOTPKG/simulation/morse-yarp``, ``$ROBOTPKG/wip/morse-pocolibs``.
