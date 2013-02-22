ROS and MORSE tutorial
======================

This is an extension of the current :doc:`tutorial <../beginner_tutorials/tutorial>`
using the ROS as our middleware (instead of raw sockets).

The builder script is the same as the one used in the sockets tutorial.
Except the part on the inteface, which in this case is ROS.
You can find it in ``examples/tutorials/tutorial-1-ros.py``.


Configuring ROS
---------------

.. code-block:: python

    pose.add_stream('ros')
    motion.add_stream('ros')


Running the simulation
----------------------

Run morse with::

    morse run examples/tutorials/tutorial-1-ros.py

In another terminal, use the `rostopic`, `rxplot` and `rxgraph`
commands to control the robot.


rostopic
--------

First, send a motion command to the robot's MotionVW actuator::

    rostopic pub -1 /atrv/motion geometry_msgs/Twist "{linear: {x: .5}, angular: {z: .5}}"

- `pub` stands for publish
- `-1` will publish 1 message
- `/atrv/motion` correspond to the topic for the motion actuator on the atrv robot
- `geometry_msgs/Twist` is the type of the message
- `"{linear: {x: .8}, angular: {z: .5}}"` the message in json
  * 0.8 m/s for `v` (linear.x)
  * 0.5 rad/s for `w` (angular.z)


Second, look at its pose::

    rostopic echo -n10 /atrv/pose

- `echo` will subscribe and print
- `-n10` will print 10 messages
- `/atrv/pose` correspond to the topic for the pose sensor on the atrv robot


rxplot
------

Once you sent a motion command, you can plot the pose in realtime with::

    rxplot /atrv/pose/pose/position/x,/atrv/pose/pose/position/y /atrv/pose/pose/orientation/z


rxgrah
------

It's an alternative to `rostopic list` showing a connexion of nodes and topics.


etc
---

For more, visit ROS wiki, where you will find many well writen
`tutorials<http://ros.org/wiki/ROS/Tutorials>`_

