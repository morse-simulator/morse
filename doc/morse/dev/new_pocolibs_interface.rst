Creation of new pocolibs middleware interfaces
==============================================

Making a new binding between a Pocolibs poster and a MORSE component can be
a bit tricky. It requires extracting the information from the poster using
Swig, extracting the required data from the data structures in the poster
and storing the information in the ``local_data`` property of the MORSE
component.

The whole process is explained using the example of a new actuator that
will read data from the **nuit** module. This module gets data from the
Kinect and publishes a poster with the position of several joints in
a human armature.

To make a new binding, you must follow these steps:


Extracting the information from the poster
------------------------------------------

- Install the modules required in your system, using
  `robotpkg <http://homepages.laas.fr/mallet/robotpkg>`_.
  This will give you access to the necessary C header files::

    $ cd ~/openrobots/robotpkg/image/niut-genom
    $ make
    $ make install

- Open the .gen file to determine the data type that will be transmitted
  inside the poster::

    $ cd work.samus.laas.fr/niut-genom-0.2
    $ less niut.gen

- Locate the poster, and check what is the name of the "data" that it uses:

.. code-block:: c

    poster Human {
        update: auto;
        data: list::list;            // <===== THIS HERE
        exec_task: NiTask;
    };

- Check the data structure used by the module, to find the data type of the
  data variable sent in the poster:

.. code-block:: c

    typedef struct NIUT_STR {
        NIUT_HUMAN_LIST list;        // <===== THIS HERE
        int verbose;
    } NIUT_STR;

- Open the include library, to look for the data types::

    $ less ~/openrobots/include/niut/niutStruct.h

- Identify the data needed, and how to get there from the structure returned
  from the poster. In this case, the poster returns a **NIUT_HUMAN_LIST**,
  and we need the **GEN_POINT_3D** ``position`` for specific joints.
  So we need to read from these data structures:

  **NIUT_HUMAN_LIST** => **NIUT_USER_STR** => **NIUT_SKELETON_STR** => **NIUT_JOINT_STR** => **GEN_POINT_3D**


The C / SWIG file
-----------------

- Copy one of the existing swig directories for other Pocolibs modules.
  The simplest example for an actuator is the ``LWR`` module for the KUKA arm::

  $ cd $MORSE_SRC/src/morse/middleware/pocolibs/actuators
  $ cp -R Lwr_poster Niut_poster
  $ cd Niut_poster

- Change the name of the files, and then edit them to change the references among them::
 
  $ cp ors_lwr_poster.[chi] ors_niut_poster.[chi]

- In the .h file, include the required header files:

.. code-block:: c

    #include <genBasic/genBasicStruct.h>
    #include <niut/niutStruct.h>

- In the .c file, call ``read_poster`` with the required data structure:

.. code-block:: c

    NIUT_HUMAN_LIST list;

    read_poster(handler, ok, &list, sizeof(NIUT_HUMAN_LIST));

- Extract the data, and return the necessary structure. In this case a GEN_POINT_3D.
  As long as the header files are included in the swig bindings, Python should
  be able to extract data from this structure:

.. code-block:: c

    joint_position = list.users[0].skeleton.joint[joint_index].position;

    return (joint_position);


The Python script
-----------------

- Copy one of the existing Python files in
  ``$MORSE_SRC/src/morse/middleware/pocolibs/[sensors|actuators]``::

  $ cd $MORSE_SRC/src/morse/middleware/pocolibs/actuators
  $ cp lwr.py niut.py

- Edit the new file and include the correct swig libraries created in the
  previous step:

.. code-block:: python

    from morse.middleware.pocolibs.actuators.Niut_Poster import ors_niut_poster

- Implement the following functions to specify the behaviour of the module:

  - ``init_extra_module``: Register the new middleware and create the connection.
    This functions remains largely the same for all modules, only necessary to
    change the name of the file ``ors_???_poster``, where ``???`` stands for
    the name of your module

  - ``init_???_poster``: (optional) Prepare the data structures necessary.
    Most commonly used in the case of sensors

  - For sensors, you must have a function ``write_???`` that copies the data
    from the ``local_data`` variable in the Blender sensor to the
    corresponding swig data structure
  
  - For actuators, you must have a function ``read_???`` that copies the data
    from the corresponding swig data structure to the ``local_data`` variable in
    the Blender actuator instance

  - The previous two functions shouls return ``True`` in case of success, or
    ``False`` otherwise


Setting up the compilation of the module
----------------------------------------

- Go to the base directory for the pocolibs middleware in MORSE::

  $ cd $MORSE_SRC/src/morse/middleware/pocolibs

- Modify the ``CMakeLists.txt``, adding entries for the new poster, in the
  **actuators_list** or **sensors_list**, accordingly. Also edit the
  **SET_MODULES_???**, listing the module and its dependencies.

- Rerun ccmake in the build directory, and ask to compile the new module too

- If everything compiles correctly, you should now be able to exchange data
  between your module and the pocolibs poster
