ROS and MORSE tutorial :tag:`ros`
=================================

This is an extension of the current :doc:`tutorial <../beginner_tutorials/tutorial>`
using the ROS as our middleware (instead of raw sockets).

The builder script is almost the same as the one used in the sockets tutorial.
The difference is in the interface, which in this case is ROS.
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

In another terminal, use the ``rostopic``, ``rqt_plot`` and ``rqt_graph``
commands to control the robot.


rostopic
--------

First, send a motion command to the robot's MotionVW actuator::

    rostopic pub -1 /atrv/motion geometry_msgs/Twist "{linear: {x: .5}, angular: {z: .5}}"

- ``pub`` stands for publish
- ``-1`` will publish 1 message
- ``/atrv/motion`` corresponds to the robots's motion controller topic
- ``geometry_msgs/Twist`` is the message's type
- ``"{linear: {x: .8}, angular: {z: .5}}"`` the message in JSON format

  - 0.8 m/s for the linear speed ``v``
  - 0.5 rad/s for the angular speed ``w``


Second, look at its pose::

    rostopic echo -n10 /atrv/pose

- ``echo`` will subscribe and print
- ``-n10`` will print 10 messages
- ``/atrv/pose`` corresponds to the robots's pose sensor topic


rqt_plot
--------

Once you send a motion command, you can plot the pose in realtime with::

    rqt_plot /atrv/pose/pose/position/x,/atrv/pose/pose/position/y /atrv/pose/pose/orientation/z

.. note:: in former ROS versions, this tool was called ``rxplot``


rqt_grah
--------

This is an alternative to ``rostopic list`` showing a collection of nodes and topics.


rospy
-----

From a terminal, start an interactive Python session by typing ``python3``
and type the following code:

.. code-block:: python

    import rospy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import Twist

    cmd = rospy.Publisher("/atrv/motion", Twist)
    motion = Twist()
    def callback(msg):
         position = msg.pose.position
         if position.x < 1:
            motion.linear.x = +0.5
         if position.x > 2:
            motion.linear.x = -0.5
         cmd.publish(motion)

    rospy.init_node("rostuto1")
    rospy.Subscriber("/atrv/pose", PoseStamped, callback)
    rospy.spin() # this will block until you press Ctrl+C

You should see the robot move forward then backward after one minute.


Notes
-----

For more information, visit the ROS wiki, where you will find many well written
`tutorials <http://ros.org/wiki/ROS/Tutorials>`_

