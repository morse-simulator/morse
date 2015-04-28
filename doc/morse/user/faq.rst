MORSE FAQ
=========

Here are some of the commonly found problems while installing or using MORSE,
and their solution:

During installation
-------------------

- **Problem**: Python executables not found when configuring using ``cmake``::

    morse/build $ cmake ..
    -- Could NOT find PythonInterp (missing:  PYTHON_EXECUTABLE) 
    CMake Error at CMakeLists.txt:24 (MESSAGE):
    Can't find python 3.3 on your system

    -- Configuring incomplete, errors occurred!

  **Solution**: Indicate to ``cmake`` the location of the Python executable
  installed in your system.  MORSE should try to find the location of the
  include files and libraries automatically::

    $ cmake -DPYTHON_EXECUTABLE=/usr/local/bin/python3.3 ..



When running morse
------------------

- **Problem**: Blender executable not found::

    * The $MORSE_BLENDER environment variable doesn't point to a Blender executable! You should fix that! Trying to look for Blender in alternative places...

    which: no blender in (/usr/lib/mpich2/bin:/usr/lib/ccache:/usr/local/bin:/bin:/usr/bin:/usr/X11R6/bin)
    * Could not find a correct Blender executable, neither in the path or in
    MORSE prefix. Blender >= 2.65 is required to run MORSE.  You can
    alternatively set the $MORSE_BLENDER environment variable to point to a
    specific Blender executable

    * Your environment is not yet correctly setup to run MORSE!
    Please fix it with above information.
    You can also run 'morse check' for more details.

  **Solution**: Configure the environment variable ``MORSE_BLENDER`` on your
  system. Locate the directory where you have installed Blender, and substitute
  that for *{path_to_blender_executable}* in the lines below. Depending on your
  shell (csh or bash) use one of these commands from your home directory.

  - csh::

    $ echo 'setenv MORSE_BLENDER {path_to_blender_executable}/blender' >> ~/.cshrc

  - bash::

    $ echo 'export MORSE_BLENDER={path_to_blender_executable}/blender' >> ~/.bashrc


- **Problem**: MORSE libraries not found::

    * We could not find the MORSE Python libraries in your system. If MORSE was
    installed to some strange location, you may want to add it to your
    PYTHONPATH. Check INSTALL for more details.
    * Your environment is not yet correctly setup to run MORSE!
    Please fix it with above information.

  **Solution**: Configure the environment variable ``PYTHONPATH`` to point to
  the directory where you have installed the MORSE libraries in your system, as
  well as the ``site-packages`` sub-directory. By default MORSE will be
  installed in ``/usr/local``, otherwise you must use the directory you
  indicated to ``cmake``.  Depending on your shell (csh or bash) use one of
  these commands from your home directory.

  - csh::

    $ echo 'setenv PYTHONPATH $PYTHONPATH\:/usr/local/lib/python3.3\:/usr/local/lib/python3.3/site-packages' >> ~/.cshrc

  - bash::

    $ echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.3:/usr/local/lib/python3.3/site-packages' >> ~/.bashrc

  .. warning::
    The name of the installation directory may be different depending on your
    Linux distribution. If you use **Ubuntu** or similar distributions, replace
    the directory name of ``python3.3/site-packages`` for
    ``python3/dist-packages``. Make sure to indicate the correct path used in
    your computer for all Python 3 libraries.


- ``urandom error`` : make sure your ``python3`` version is equal to the one in
  Blender.

- :tag:`ros` ``No module named 'rospkg'`` : install ``rospkg`` with Python 3
  support, *ie* ``sudo apt-get install python3-rospkg``

- :tag:`ros` ``No module named 'roslib'`` : ``source /opt/ros/***/setup.bash``.

- ``No module named 'error'`` : ``Yaml`` for Python2 is running instead of
  Python 3, install ``python3-yaml`` and/or fix your ``PYTHONPATH``.

Developer specific
------------------

- **Problem**: No rule to make target 'test'

  * When trying to run ``make test``, you get this error and make stops::

      morse/build $ make test
      make: *** No rule to make target 'test'. Stop.

  * This is because the ``pymorse`` bindings have not been installed

  **Solution**: Reconfigure MORSE with the ``PYMORSE_SUPPORT`` option, either
  by selecting the option using ``ccmake`` or by running::

    $ cd [your morse source directory]/build
    $ cmake -DPYMORSE_SUPPORT=ON [other options] ..

  You then need to run::

    $ make install
    $ make rebuild_cache

  And finally::

    $ make test


- **Problem**: pymorse not found. When running ``make rebuild_cache``, there is
  an error indicating::

      morse/build $ make rebuild_cache
      Running CMake to regenerate build system...
      \-\- will install python files in [your installation directory]
      CMake Warning at CMakeLists.txt:116 (MESSAGE):
            Can't run test, pymorse is required but cannot be found.
            You may want to install it first.

  **Solution**: The ``PYTHONPATH`` variable is not correctly setup. The location
  where ``pymorse.py`` was installed must be included in ``PYTHONPATH``.
  The directory is indicated in the error message
