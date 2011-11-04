MORSE FAQ
=========

Here are some of the commonly found problems while installing or using MORSE, and their solution:

During installation
-------------------

- **Problem**: Python libraries not found::

    morse/_build $ cmake ..
    -- Could NOT find Python3Libs (missing:  PYTHON3_LIBRARIES) 
    -- will install python files in /usr/local/lib/python3.2/site-packages
    -- Can't find sphinx-build : will not build the documentation
    CMake Error: The following variables are used in this project, but they are set to NOTFOUND.
    Please set them or make sure they are set and tested correctly in the CMake files:
    PYTHON3_LIBRARY (ADVANCED)

        -- Configuring incomplete, errors occurred!

  **Solution**: Indicate the location of the Python libraries as installed in your system::

    $ cmake -DPYTHON3_INCLUDE_PATH=/usr/local/include/python3.2mu -DPYTHON3_LIBRARY=/usr/local/lib/python3.2mu.a ..



When running morse
------------------

- **Problem**: Blender not found::

    * The $MORSE_BLENDER environment variable doesn't point to a Blender executable! You should fix that! Trying to look for Blender in alternative places...

    which: no blender in (/home/gechever/openrobots/bin:/usr/lib/mpich2/bin:/usr/lib/ccache:/home/gechever:/home/gechever/bin:/usr/local/bin:/bin:/usr/bin:/usr/X11R6/bin:/home/gechever/bin)
    * Could not find a correct Blender executable, neither in the path or in MORSE
    prefix. Blender >= 2.59 and < 3 is required to run MORSE.
    You can alternatively set the $MORSE_BLENDER environment variable to point to
    a specific Blender executable

    * Your environment is not yet correctly setup to run MORSE!
    Please fix it with above informations.
    You can also run 'morse check' for more details.

  **Solution**: Configure the environment variable ``MORSE_BLENDER`` on your system. Depending on your shell (csh or bash) use one of these commands::

    $ setenv MORSE_BLENDER {path_to_blender_executable}/blender  (csh)
    $ export MORSE_BLENDER={path_to_blender_executable}/blender  (bash)




- **Problem**: MORSE libraries not found::

    * We could not find the MORSE Python libraries in your system. If MORSE was
    installed to some strange location, you may want to add it to your
    PYTHONPATH. Check INSTALL for more details.
    * Your environment is not yet correctly setup to run MORSE!
    Please fix it with above informations.

  **Solution**: Configure the environment variable ``PYTHONPATH`` to point to the directory where Python3 is installed in your system, as well as the ``site-packages`` directory::

    $setenv PYTHONPATH /usr/local/lib/python3.2\:/usr/local/lib/python3.2/site-packages   (csh)
    $export PYTHONPATH=/usr/local/lib/python3.2:/usr/local/lib/python3.2/site-packages    (bash)
