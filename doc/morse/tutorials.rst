Tutorials 
=========

Welcome in the MORSE tutorials section!

As an academic simulator for robotics, MORSE is a large piece of software which
may take some time to master.

However, you should hopefully be able to get your first
simulation running very quickly.

If you haven't yet read it, we recommend you to first read the
:doc:`Quickstart<quickstart>` page and come back here afterwards.


What is the general workflow?
-----------------------------

MORSE relies on the Blender 3D package to run your simulation. To create a
simulation scene, you need to describe it using the **Builder** python API,
which allows you to simply describe simulation scenes. More about that in a
moment.

Once you have a description of your simulation, you can run it:: 

 $ morse run <your file>

Now, instruct your :doc:`middleware (ROS, YARP,...)<user/integration>` to address
the simulator instead of the real robot, and start your control softwares as
you would do on a real robot. If you are not relying on an explicit middleware,
you can also use the :doc:`socket interface <user/middlewares/socket>` :tag:`socket`
or the :doc:`Python API <pymorse>` :tag:`pymorse`. MORSE supports the two
classic interaction mechanisms provided by most robotic middleware:

1. Using **RPC**-oriented calls :tag:`service`
2. Using **stream**-oriented interfaces. :tag:`datastream`

.. note::

    In various places in the documentation, you will see labels like
    :tag:`builder`, :tag:`ros` or :tag:`service`. They denote the main subjects
    that the section or the tutorial deal with.

The first simulation
--------------------

First of all, :doc:`install MORSE<user/installation>` if you haven't already
done so.

Next step, :doc:`user/beginner_tutorials/tutorial`. :tag:`builder`
:tag:`socket`

Basic MORSE user interface
--------------------------

The next tutorial describes how to launch the MORSE simulation, and
how to do basic interaction with it.


.. toctree::
    :maxdepth: 1

    user/basic_morse
    

Beginners tutorials
-------------------

This section shows how MORSE can interact with different middlewares.

.. toctree::
    :maxdepth: 1

    user/beginner_tutorials/tutorial
    user/beginner_tutorials/yarp_tutorial
    user/beginner_tutorials/ros_tutorial
    user/beginner_tutorials/moos_tutorial
    user/beginner_tutorials/hri_tutorial

Intermediate tutorials
----------------------

These tutorials provide more in-depth explanations of how to setup simulations
with specific requirements.

.. toctree::
    :maxdepth: 1

    user/advanced_tutorials/a_journey_to_a_new_simulation
    user/advanced_tutorials/cat_and_mouse
    user/advanced_tutorials/flying_cat_and_mouse
    user/advanced_tutorials/ros_nav_tutorial
    user/advanced_tutorials/mocap_tutorial
    user/advanced_tutorials/request_tutorial
    user/advanced_tutorials/noise_ghost_tutorial
    user/advanced_tutorials/hla_tutorial
    user/screencast
    
Multi-node tutorials
--------------------

These tutorials help setting up a multi-node simulation environment.

.. toctree::
    :glob:
    :maxdepth: 1

    user/multinode/tutorials/*


Blender specific instructions
-----------------------------

Since MORSE is completely dependent on Blender, it is important to know some of
Blender's functionality.

.. toctree::
    :maxdepth: 1

    user/blender_tutorials/basic_blender
    user/blender_tutorials/advanced_blender
