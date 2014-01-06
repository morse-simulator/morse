ROS
===

Installation
------------

Please follow the instructions in the :doc:`installation procedure  <../installation/mw/ros>`.

Generation of ROS-node and topics
----------------------------------

The ROS middleware creates one ROS-node called "morse" and on ROS topic for
every sensor and every actuator. The names of the ROS-topics are generated in
the following way: ``<name_of_parent_blender_object>/<name_of_blender_object>``.

For instance, if you have an odometry sensor called "odometry" on a robot
called "atrv", the ROS Odometry messages will be published on ``/atrv/odometry``.

.. _ros_ds_configuration:

Configuration specificities
---------------------------

When configuring a component to export its data through ROS, you can pass
the option ``topic`` to define the name of the topic exported by the
component.


.. code-block :: python

    foo.add_stream('ros', topic = '/robot/myTopic')


The same way, you can set a custom ``frame_id`` and ``child_frame_id``:

.. code-block :: python

    foo.add_stream('ros', frame_id = '/world', child_frame_id = '/footprint')
