MORSE FAQ
=========

Here are some of the commonly found problems while installing or using MORSE, and their solution:

During installation
-------------------

- **Problem**: Python libraries not found when configuring using ``cmake``::

    morse/_build $ cmake ..
    -- Could NOT find Python3Libs (missing:  PYTHON3_LIBRARIES) 
    -- will install python files in /usr/local/lib/python3.2/site-packages
    -- Can't find sphinx-build : will not build the documentation
    CMake Error: The following variables are used in this project, but they are set to NOTFOUND.
    Please set them or make sure they are set and tested correctly in the CMake files:
    PYTHON3_LIBRARY (ADVANCED)

        -- Configuring incomplete, errors occurred!

  **Solution**: Indicate to ``cmake`` the location of the Python libraries as installed in your system::

    $ cmake -DPYTHON3_INCLUDE_PATH=/usr/local/include/python3.2mu -DPYTHON3_LIBRARY=/usr/local/lib/python3.2mu.a ..



When running morse
------------------

- **Problem**: Blender executable not found::

    * The $MORSE_BLENDER environment variable doesn't point to a Blender executable! You should fix that! Trying to look for Blender in alternative places...

    which: no blender in (/usr/lib/mpich2/bin:/usr/lib/ccache:/usr/local/bin:/bin:/usr/bin:/usr/X11R6/bin)
    * Could not find a correct Blender executable, neither in the path or in MORSE
    prefix. Blender >= 2.59 and < 2.62 is required to run MORSE.
    You can alternatively set the $MORSE_BLENDER environment variable to point to
    a specific Blender executable

    * Your environment is not yet correctly setup to run MORSE!
    Please fix it with above informations.
    You can also run 'morse check' for more details.

  **Solution**: Configure the environment variable ``MORSE_BLENDER`` on your system. Locate the directory where you have installed Blender, and substitute that for *{path_to_blender_executable}* in the lines below. Depending on your shell (csh or bash) use one of these commands from your home directory.

  - csh::

    $ echo 'setenv MORSE_BLENDER {path_to_blender_executable}/blender' >> ~/.cshrc

  - bash::

    $ echo 'export MORSE_BLENDER={path_to_blender_executable}/blender' >> ~/.bashrc


- **Problem**: MORSE libraries not found::

    * We could not find the MORSE Python libraries in your system. If MORSE was
    installed to some strange location, you may want to add it to your
    PYTHONPATH. Check INSTALL for more details.
    * Your environment is not yet correctly setup to run MORSE!
    Please fix it with above informations.

  **Solution**: Configure the environment variable ``PYTHONPATH`` to point to the directory where you have installed the MORSE libraries in your system, as well as the ``site-packages`` sub-directory. By default MORSE will be installed in ``/usr/local``, otherwise you must use the directory you indicated to ``cmake``.
  Depending on your shell (csh or bash) use one of these commands from your home directory.

  - csh::

    $ echo 'setenv PYTHONPATH $PYTHONPATH\:/usr/local/lib/python3.2\:/usr/local/lib/python3.2/site-packages' >> ~/.cshrc

  - bash::

    $ echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.2:/usr/local/lib/python3.2/site-packages' >> ~/.bashrc

  .. warning::
    The name of the installation directory may be different depending on your Linux distribution. If you use **Ubuntu** or similar distributions, replace the directory name of ``python3.2/site-packages`` for ``python3/dist-packages``. Make sure to indicate the correct path used in your computer for all Python 3 libraries.
